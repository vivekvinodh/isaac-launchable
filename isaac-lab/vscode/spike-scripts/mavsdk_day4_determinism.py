#!/usr/bin/env python3
"""
Day 4 determinism measurement (v2 — 1s averaged end-sample per runbook spec).

Scenario: arm → takeoff to 10m → hover 50s → sample over final 1s (averaged)
→ land → disarm.

Captures averaged final position (NED) + averaged final attitude (Euler) +
flight_time + sample count into /tmp/day4_<label>.json. Run twice with a fresh
Pegasus + PX4 boot between runs; diff with mavsdk_day4_diff.py.

Usage:
    python3 mavsdk_day4_determinism.py run1
    # ... fresh boot Pegasus + PX4 ...
    python3 mavsdk_day4_determinism.py run2
    python3 mavsdk_day4_diff.py /tmp/day4_run1.json /tmp/day4_run2.json
"""

import asyncio
import json
import math
import statistics
import sys
import time

from mavsdk import System


HOVER_ALT_M = 5.0  # Reduced from runbook's 10m: Pegasus warehouse has a ~9m ceiling
ASCENT_S = 10      # 5m ascent is faster than 10m
HOVER_S = 50
SAMPLE_S = 1.0


def _wrap_pm180(angle_deg: float) -> float:
    """Wrap an angle to (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _circular_mean_deg(angles_deg):
    """Mean of angles handling wrap-around (e.g. averaging 179° and -179°)."""
    if not angles_deg:
        return float("nan")
    sin_sum = sum(math.sin(math.radians(a)) for a in angles_deg)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(sin_sum / len(angles_deg), cos_sum / len(angles_deg)))


async def _collect_position(drone, samples: list, deadline: float) -> None:
    async for pos in drone.telemetry.position_velocity_ned():
        samples.append((pos.position.north_m, pos.position.east_m, pos.position.down_m))
        if time.monotonic() >= deadline:
            return


async def _collect_attitude(drone, samples: list, deadline: float) -> None:
    async for att in drone.telemetry.attitude_euler():
        samples.append((att.roll_deg, att.pitch_deg, att.yaw_deg))
        if time.monotonic() >= deadline:
            return


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

    await drone.action.arm()
    print(f"[{label}] Armed.")

    await drone.action.set_takeoff_altitude(HOVER_ALT_M)
    takeoff_wallclock = time.time()
    await drone.action.takeoff()
    print(f"[{label}] Takeoff to {HOVER_ALT_M} m.")

    print(f"[{label}] Ascent + stabilization {ASCENT_S}s...")
    await asyncio.sleep(ASCENT_S)

    print(f"[{label}] Hovering {HOVER_S}s...")
    await asyncio.sleep(HOVER_S)

    # 1s end-sample window: collect all available frames concurrently.
    pos_samples: list = []
    att_samples: list = []
    deadline = time.monotonic() + SAMPLE_S
    print(f"[{label}] Sampling over final {SAMPLE_S}s...")
    await asyncio.gather(
        _collect_position(drone, pos_samples, deadline),
        _collect_attitude(drone, att_samples, deadline),
    )

    if not pos_samples or not att_samples:
        raise RuntimeError(f"[{label}] No telemetry frames received during sample window")

    avg_n = statistics.fmean(s[0] for s in pos_samples)
    avg_e = statistics.fmean(s[1] for s in pos_samples)
    avg_d = statistics.fmean(s[2] for s in pos_samples)

    avg_roll = _circular_mean_deg([s[0] for s in att_samples])
    avg_pitch = _circular_mean_deg([s[1] for s in att_samples])
    avg_yaw = _circular_mean_deg([s[2] for s in att_samples])

    flight_time = time.time() - takeoff_wallclock

    result = {
        "label": label,
        "timestamp_unix": time.time(),
        "scenario": {
            "hover_altitude_m": HOVER_ALT_M,
            "ascent_s": ASCENT_S,
            "hover_s": HOVER_S,
            "sample_window_s": SAMPLE_S,
        },
        "samples": {
            "position_count": len(pos_samples),
            "attitude_count": len(att_samples),
        },
        "final_position_ned": {
            "north_m": avg_n,
            "east_m": avg_e,
            "down_m": avg_d,
        },
        "final_position_ned_stdev": {
            "north_m": statistics.pstdev(s[0] for s in pos_samples) if len(pos_samples) > 1 else 0.0,
            "east_m": statistics.pstdev(s[1] for s in pos_samples) if len(pos_samples) > 1 else 0.0,
            "down_m": statistics.pstdev(s[2] for s in pos_samples) if len(pos_samples) > 1 else 0.0,
        },
        "final_attitude_deg": {
            "roll_deg": avg_roll,
            "pitch_deg": avg_pitch,
            "yaw_deg": avg_yaw,
        },
        "final_attitude_deg_stdev": {
            "roll_deg": statistics.pstdev(s[0] for s in att_samples) if len(att_samples) > 1 else 0.0,
            "pitch_deg": statistics.pstdev(s[1] for s in att_samples) if len(att_samples) > 1 else 0.0,
            "yaw_deg": statistics.pstdev(s[2] for s in att_samples) if len(att_samples) > 1 else 0.0,
        },
        "flight_time_s_wallclock": flight_time,
    }

    out_path = f"/tmp/day4_{label}.json"
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2, sort_keys=True)
    print(f"[{label}] Sampled state saved to {out_path}")
    print(json.dumps(result, indent=2, sort_keys=True))

    print(f"[{label}] Landing...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    await drone.action.disarm()
    print(f"[{label}] Done.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: mavsdk_day4_determinism.py <label>", file=sys.stderr)
        sys.exit(1)
    asyncio.run(run(sys.argv[1]))
