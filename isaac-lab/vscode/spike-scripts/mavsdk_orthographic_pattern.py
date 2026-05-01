#!/usr/bin/env python3
"""
Orthographic survey pattern with deterministic-test instrumentation.

Flies a small lawnmower/serpentine grid at fixed altitude — the canonical
flight pattern for drone-based orthomosaic mapping. Sized to fit inside
Pegasus's default warehouse scene (well under the ~9m ceiling, small
footprint, returns to start).

This script collects continuous position+velocity telemetry throughout
the mission, detects per-waypoint settling, and writes a structured
JSON record of timing, position errors, and path metrics. Two runs with
fresh PX4+Pegasus state should produce nearly-identical metrics — drift
is the deterministic-failure signal.

Pattern (NED relative to takeoff origin):
    Line 1: north pass (0,0) → (4,0)
    East shift: (4,0) → (4, 1.5)
    Line 2: south pass (4, 1.5) → (0, 1.5)
    East shift: (0, 1.5) → (0, 3)
    ... 5 lines covering 4m × 6m at 4m altitude
    Return to origin (0, 0) before landing

Body yaw stays at 0° throughout — strafes east at line-ends without
rotating, mimicking gimbal-nadir ortho mission semantics.

Usage:
    python3 mavsdk_orthographic_pattern.py [label]

If `label` is omitted, defaults to "ortho_<unix_ts>". Output is written
to /tmp/ortho_<label>.json. Diff two runs with `mavsdk_day4_diff.py`
or any json diff tool — key fields for determinism are
`waypoints[*].settled_position_ned` and `waypoints[*].arrival_dt_s`.

Prerequisites:
- PX4 SITL running (Pegasus's canonical args, SIM_MODEL=gazebo-classic_iris).
- Pegasus loaded with warehouse scene + iris airframe, **Play pressed**
  (see spike RUNBOOK errata #20 — Play is what triggers TCP accept).
- MAVSDK reachable on udpin://0.0.0.0:14540 (PX4 default).
"""

import asyncio
import json
import math
import statistics
import sys
import time
from typing import List, Optional, Tuple

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ---- Pattern parameters (warehouse-safe defaults) ----
ALTITUDE_M = 4.0
NORTH_LENGTH_M = 4.0
EAST_WIDTH_M = 6.0
LINE_SPACING_M = 1.5
DWELL_SECONDS = 3.0          # was 2.0 — extended for cleaner settling under measurement
TAKEOFF_SETTLE_SECONDS = 3.0

# ---- Settling detection ----
SETTLED_DIST_M = 0.10        # within 10 cm of commanded
SETTLED_SPEED_M_S = 0.10     # horizontal speed under 10 cm/s

EPS = 1e-6


# A single telemetry frame: (t_mono, n, e, d, vn, ve, vd)
TelemetryFrame = Tuple[float, float, float, float, float, float, float]


def build_pattern() -> List[PositionNedYaw]:
    """Compute NED waypoints for a serpentine grid at fixed altitude."""
    down = -ALTITUDE_M
    waypoints = [PositionNedYaw(0.0, 0.0, down, 0.0)]

    east = 0.0
    direction = +1
    while east <= EAST_WIDTH_M + EPS:
        far_north = NORTH_LENGTH_M if direction == +1 else 0.0
        waypoints.append(PositionNedYaw(far_north, east, down, 0.0))

        next_east = east + LINE_SPACING_M
        if next_east > EAST_WIDTH_M + EPS:
            break
        waypoints.append(PositionNedYaw(far_north, next_east, down, 0.0))
        east = next_east
        direction = -direction

    waypoints.append(PositionNedYaw(0.0, 0.0, down, 0.0))
    return waypoints


class TelemetryCollector:
    """Background-collects (time, position, velocity) frames into a list.

    Single MAVSDK subscription kept alive for the entire mission. The
    collected frames feed both real-time settling detection and post-mission
    metric computation.
    """

    def __init__(self) -> None:
        self.frames: List[TelemetryFrame] = []
        self._task: Optional[asyncio.Task] = None
        self._stop = False

    def start(self, drone: System) -> None:
        self._task = asyncio.create_task(self._run(drone))

    async def _run(self, drone: System) -> None:
        try:
            async for pos in drone.telemetry.position_velocity_ned():
                if self._stop:
                    return
                self.frames.append((
                    time.monotonic(),
                    pos.position.north_m,
                    pos.position.east_m,
                    pos.position.down_m,
                    pos.velocity.north_m_s,
                    pos.velocity.east_m_s,
                    pos.velocity.down_m_s,
                ))
        except asyncio.CancelledError:
            return

    async def stop(self) -> None:
        self._stop = True
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass


def _euclid_3d(a, b) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _horiz_speed(frame: TelemetryFrame) -> float:
    _, _, _, _, vn, ve, _ = frame
    return math.hypot(vn, ve)


def find_settled_frame(
    frames: List[TelemetryFrame],
    commanded_ned: Tuple[float, float, float],
    setpoint_t_mono: float,
) -> Optional[int]:
    """Return index of the first frame after setpoint where the drone is
    within SETTLED_DIST_M of commanded AND moving slower than
    SETTLED_SPEED_M_S. None if never settled before the next setpoint.
    """
    for i, frame in enumerate(frames):
        t_mono, n, e, d, _, _, _ = frame
        if t_mono < setpoint_t_mono:
            continue
        dist = _euclid_3d((n, e, d), commanded_ned)
        if dist <= SETTLED_DIST_M and _horiz_speed(frame) <= SETTLED_SPEED_M_S:
            return i
    return None


def integrate_path_length(
    frames: List[TelemetryFrame], t_start: float, t_end: float
) -> float:
    """Sum Euclidean deltas between consecutive frames within [t_start, t_end]."""
    total = 0.0
    prev: Optional[Tuple[float, float, float]] = None
    for t_mono, n, e, d, *_ in frames:
        if t_mono < t_start or t_mono > t_end:
            continue
        cur = (n, e, d)
        if prev is not None:
            total += _euclid_3d(prev, cur)
        prev = cur
    return total


def dwell_position_stdev(
    frames: List[TelemetryFrame], settled_t: float, dwell_end_t: float
) -> Tuple[float, float, float]:
    """Stdev of (n, e, d) over the dwell window after settling. Tells us
    how much the drone wandered while station-keeping."""
    in_window = [
        (n, e, d) for (t, n, e, d, *_) in frames
        if settled_t <= t <= dwell_end_t
    ]
    if len(in_window) < 2:
        return (0.0, 0.0, 0.0)
    return (
        statistics.pstdev(p[0] for p in in_window),
        statistics.pstdev(p[1] for p in in_window),
        statistics.pstdev(p[2] for p in in_window),
    )


def settled_position_average(
    frames: List[TelemetryFrame], settled_t: float, dwell_end_t: float
) -> Tuple[float, float, float]:
    """Average position over the settled window."""
    in_window = [
        (n, e, d) for (t, n, e, d, *_) in frames
        if settled_t <= t <= dwell_end_t
    ]
    if not in_window:
        return (float("nan"), float("nan"), float("nan"))
    return (
        statistics.fmean(p[0] for p in in_window),
        statistics.fmean(p[1] for p in in_window),
        statistics.fmean(p[2] for p in in_window),
    )


async def run(label: str) -> None:
    drone = System()
    print(f"[{label}] Connecting to udpin://0.0.0.0:14540...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{label}] Connected.")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[{label}] Position estimate OK.")
            break

    collector = TelemetryCollector()
    collector.start(drone)
    await asyncio.sleep(0.2)  # let first frame land before arming

    events = {}

    print(f"[{label}] Arming")
    events["arm_unix_s"] = time.time()
    await drone.action.arm()

    print(f"[{label}] Takeoff to {ALTITUDE_M:.1f} m")
    await drone.action.set_takeoff_altitude(ALTITUDE_M)
    events["takeoff_unix_s"] = time.time()
    await drone.action.takeoff()

    async for in_air in drone.telemetry.in_air():
        if in_air:
            break
    events["first_airborne_unix_s"] = time.time()
    print(f"[{label}] Airborne. Settling for {TAKEOFF_SETTLE_SECONDS:.0f}s")
    await asyncio.sleep(TAKEOFF_SETTLE_SECONDS)

    waypoints = build_pattern()
    print(f"[{label}] Pattern: {len(waypoints)} waypoints, "
          f"{NORTH_LENGTH_M:.1f}m × {EAST_WIDTH_M:.1f}m at {ALTITUDE_M:.1f}m AGL")

    # Per-waypoint timing record. We reconstruct settled-state from the
    # telemetry log post-mission — keeps the live loop simple and
    # avoids time-sensitive synchronization with the subscription.
    waypoint_log = []  # list of dicts

    print(f"[{label}] Entering offboard")
    try:
        await drone.offboard.set_position_ned(waypoints[0])
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[{label}] !! Offboard start failed: {e}")
        await drone.action.land()
        await collector.stop()
        return

    for i, wp in enumerate(waypoints):
        sent_t_mono = time.monotonic()
        sent_unix = time.time()
        await drone.offboard.set_position_ned(wp)
        await asyncio.sleep(DWELL_SECONDS)
        dwell_end_t_mono = time.monotonic()
        waypoint_log.append({
            "index": i,
            "commanded_ned": [wp.north_m, wp.east_m, wp.down_m],
            "yaw_deg": wp.yaw_deg,
            "setpoint_sent_t_mono_s": sent_t_mono,
            "setpoint_sent_unix_s": sent_unix,
            "dwell_end_t_mono_s": dwell_end_t_mono,
        })
        print(f"[{label}] WP {i+1:2}/{len(waypoints)}: "
              f"N={wp.north_m:+.1f} E={wp.east_m:+.1f} D={wp.down_m:+.1f}")

    print(f"[{label}] Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        print(f"[{label}] !! Offboard stop failed: {e}")

    print(f"[{label}] Landing")
    events["land_command_unix_s"] = time.time()
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    events["touchdown_unix_s"] = time.time()
    print(f"[{label}] Disarming")
    await drone.action.disarm()
    events["disarm_unix_s"] = time.time()

    # End telemetry capture before post-processing.
    await collector.stop()
    frames = collector.frames

    # ---- Post-processing ----
    print(f"[{label}] Post-processing {len(frames)} telemetry frames...")

    waypoints_out = []
    leg_actual_total_m = 0.0
    leg_commanded_total_m = 0.0
    legs_out = []
    errors_m: List[float] = []
    settled_count = 0

    prev_commanded: Optional[Tuple[float, float, float]] = None
    prev_settled_t: Optional[float] = None

    for entry in waypoint_log:
        cmd = tuple(entry["commanded_ned"])
        sent_t = entry["setpoint_sent_t_mono_s"]
        dwell_end_t = entry["dwell_end_t_mono_s"]

        settled_idx = find_settled_frame(frames, cmd, sent_t)
        if settled_idx is not None:
            settled_t = frames[settled_idx][0]
            arrival_dt = settled_t - sent_t
            settled_pos = settled_position_average(frames, settled_t, dwell_end_t)
            err = _euclid_3d(settled_pos, cmd)
            stdev = dwell_position_stdev(frames, settled_t, dwell_end_t)
            settled_count += 1
            errors_m.append(err)
            wp_record = {
                **entry,
                "settled_t_mono_s": settled_t,
                "arrival_dt_s": arrival_dt,
                "settled_position_ned": list(settled_pos),
                "position_error_m": err,
                "dwell_position_stdev_ned_m": list(stdev),
            }
        else:
            wp_record = {
                **entry,
                "settled_t_mono_s": None,
                "arrival_dt_s": None,
                "settled_position_ned": None,
                "position_error_m": None,
                "dwell_position_stdev_ned_m": None,
            }

        # Per-leg metrics — leg = transit from previous-commanded to current-commanded.
        if prev_commanded is not None and prev_settled_t is not None and \
                wp_record["settled_t_mono_s"] is not None:
            leg_commanded = _euclid_3d(prev_commanded, cmd)
            leg_actual = integrate_path_length(
                frames, prev_settled_t, wp_record["settled_t_mono_s"])
            duration = wp_record["settled_t_mono_s"] - prev_settled_t
            legs_out.append({
                "from_index": entry["index"] - 1,
                "to_index": entry["index"],
                "commanded_distance_m": leg_commanded,
                "actual_distance_m": leg_actual,
                "duration_s": duration,
                "mean_speed_m_s": leg_actual / duration if duration > 0 else 0.0,
            })
            leg_commanded_total_m += leg_commanded
            leg_actual_total_m += leg_actual

        prev_commanded = cmd
        if wp_record["settled_t_mono_s"] is not None:
            prev_settled_t = wp_record["settled_t_mono_s"]
        waypoints_out.append(wp_record)

    summary = {
        "waypoints_total": len(waypoint_log),
        "waypoints_settled": settled_count,
        "max_position_error_m": max(errors_m) if errors_m else None,
        "mean_position_error_m": statistics.fmean(errors_m) if errors_m else None,
        "total_path_m_actual": leg_actual_total_m,
        "total_path_m_commanded": leg_commanded_total_m,
        "path_efficiency": (
            leg_actual_total_m / leg_commanded_total_m
            if leg_commanded_total_m > 0 else None
        ),
        "mission_total_wallclock_s": (
            events["disarm_unix_s"] - events["arm_unix_s"]
        ),
    }

    result = {
        "label": label,
        "timestamp_unix": time.time(),
        "scenario": {
            "altitude_m": ALTITUDE_M,
            "north_length_m": NORTH_LENGTH_M,
            "east_width_m": EAST_WIDTH_M,
            "line_spacing_m": LINE_SPACING_M,
            "dwell_seconds": DWELL_SECONDS,
            "settled_dist_m": SETTLED_DIST_M,
            "settled_speed_m_s": SETTLED_SPEED_M_S,
        },
        "events": events,
        "telemetry_frames_collected": len(frames),
        "waypoints": waypoints_out,
        "legs": legs_out,
        "summary": summary,
    }

    out_path = f"/tmp/ortho_{label}.json"
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2, sort_keys=True)
    print(f"[{label}] Result saved to {out_path}")
    print()
    print(f"[{label}] === Summary ===")
    print(f"[{label}]   Waypoints settled:   "
          f"{settled_count}/{len(waypoint_log)}")
    if errors_m:
        print(f"[{label}]   Max position error:  {max(errors_m):.4f} m")
        print(f"[{label}]   Mean position error: {statistics.fmean(errors_m):.4f} m")
    print(f"[{label}]   Path: {leg_actual_total_m:.2f} m actual / "
          f"{leg_commanded_total_m:.2f} m commanded "
          f"(efficiency {summary['path_efficiency']:.3f})"
          if summary["path_efficiency"] is not None else
          f"[{label}]   Path: {leg_actual_total_m:.2f} m actual / "
          f"{leg_commanded_total_m:.2f} m commanded")
    print(f"[{label}]   Mission wallclock:   "
          f"{summary['mission_total_wallclock_s']:.2f} s")


if __name__ == "__main__":
    label = sys.argv[1] if len(sys.argv) > 1 else f"run_{int(time.time())}"
    try:
        asyncio.run(run(label))
    except KeyboardInterrupt:
        print("\n-- Interrupted!")
        sys.exit(1)
