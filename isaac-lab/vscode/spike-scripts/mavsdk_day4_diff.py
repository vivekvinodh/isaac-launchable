#!/usr/bin/env python3
"""
Day 4 drift computation.

Reads two run JSONs produced by mavsdk_day4_determinism.py and prints:
- Position drift (Euclidean distance, meters)
- Max attitude drift (degrees, max of |roll|, |pitch|, |yaw| deltas)
- Pass/fail vs runbook bar (0.05 m / 1°)

Usage:
    python3 mavsdk_day4_diff.py /tmp/day4_run1.json /tmp/day4_run2.json
"""

import json
import math
import sys


POS_BAR_M = 0.05
ATT_BAR_DEG = 1.0


def main(path_a: str, path_b: str) -> int:
    a = json.load(open(path_a))
    b = json.load(open(path_b))

    pa = a["final_position_ned"]
    pb = b["final_position_ned"]
    dn = pa["north_m"] - pb["north_m"]
    de = pa["east_m"] - pb["east_m"]
    dd = pa["down_m"] - pb["down_m"]
    pos_drift = math.sqrt(dn * dn + de * de + dd * dd)

    aa = a["final_attitude_deg"]
    ab = b["final_attitude_deg"]
    droll = abs(aa["roll_deg"] - ab["roll_deg"])
    dpitch = abs(aa["pitch_deg"] - ab["pitch_deg"])
    dyaw = abs(aa["yaw_deg"] - ab["yaw_deg"])
    att_drift = max(droll, dpitch, dyaw)

    print("=" * 60)
    print(f"Day 4 drift: {a['label']} vs {b['label']}")
    print("=" * 60)
    print(f"Position drift (Euclidean m): {pos_drift:.6f}    bar < {POS_BAR_M}")
    print(f"  Δnorth: {dn:+.6f} m")
    print(f"  Δeast:  {de:+.6f} m")
    print(f"  Δdown:  {dd:+.6f} m")
    print()
    print(f"Max attitude drift (deg):    {att_drift:.6f}    bar < {ATT_BAR_DEG}")
    print(f"  Δroll:  {droll:.6f}°")
    print(f"  Δpitch: {dpitch:.6f}°")
    print(f"  Δyaw:   {dyaw:.6f}°")
    print()
    print(
        f"Flight time: A={a['flight_time_s_wallclock']:.3f}s  "
        f"B={b['flight_time_s_wallclock']:.3f}s  "
        f"Δ={a['flight_time_s_wallclock'] - b['flight_time_s_wallclock']:+.3f}s"
    )
    print()

    pos_pass = pos_drift < POS_BAR_M
    att_pass = att_drift < ATT_BAR_DEG
    overall = pos_pass and att_pass

    print(f"Position bar: {'PASS' if pos_pass else 'FAIL'}")
    print(f"Attitude bar: {'PASS' if att_pass else 'FAIL'}")
    print(f"Day 4 verdict: {'PASS' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: mavsdk_day4_diff.py <run1.json> <run2.json>", file=sys.stderr)
        sys.exit(1)
    sys.exit(main(sys.argv[1], sys.argv[2]))
