#!/usr/bin/env python3
"""One-shot JAKA joint reader.

Teach the arm to a target pose with the pendant, then run this to print
the current joint values in both radians and degrees. Copy the values
into `phase_j5_deg` etc. for the initial scan.

Usage:
    python3 read_joints.py                       # default IP 10.5.5.100
    python3 read_joints.py --ip 10.5.5.100

The script only calls `login` + `get_joint_position` + `logout`, so it
does not power on or enable the robot. Safe to run while the arm is in
manual/teach mode on the pendant.
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import platform
import sys


def _init_sdk_env() -> None:
    if platform.system() != "Linux":
        raise RuntimeError("Linux only")
    candidates = [
        os.environ.get("JAKA_SDK_ROOT", "").strip(),
        "/opt/jaka_sdk",
        os.path.expanduser(
            "~/project/450_g12/JAKA-A5+UR5e/JAKA_A5_Docs/JAKA SDK v2.2.2"
        ),
    ]
    for cand in candidates:
        if not cand:
            continue
        so = os.path.join(
            cand, "04 Linux", "python3", "x86_64-linux-gnu", "libjakaAPI.so"
        )
        if os.path.isfile(so):
            ctypes.CDLL(so)
            py_path = os.path.join(
                cand, "04 Linux", "python3", "x86_64-linux-gnu"
            )
            if py_path not in sys.path:
                sys.path.insert(0, py_path)
            return
    raise FileNotFoundError(
        f"JAKA SDK not found; tried: {[c for c in candidates if c]}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.5.5.100", help="JAKA controller IP")
    args = parser.parse_args()

    _init_sdk_env()
    import jkrc  # noqa: E402

    rc = jkrc.RC(args.ip)
    print(f"login    -> {rc.login()}")
    try:
        ret = rc.get_joint_position()
        if not isinstance(ret, (tuple, list)) or ret[0] != 0:
            print(f"get_joint_position failed: {ret}", file=sys.stderr)
            return 1
        joints_rad = list(ret[1])
        print()
        print("=" * 60)
        print("current joint values")
        print("=" * 60)
        for i, q in enumerate(joints_rad, start=1):
            print(f"  J{i}:  {q:+9.5f} rad   ({math.degrees(q):+8.3f} deg)")
        print("-" * 60)
        print("as list (rad):")
        print("  [" + ", ".join(f"{q:+.5f}" for q in joints_rad) + "]")
        print("as list (deg):")
        print("  [" + ", ".join(f"{math.degrees(q):+.3f}" for q in joints_rad) + "]")
        print("=" * 60)
        return 0
    finally:
        try:
            rc.logout()
        except Exception as exc:  # noqa: BLE001
            print(f"logout error (ignored): {exc}", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
