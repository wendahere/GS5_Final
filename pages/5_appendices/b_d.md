---
title: "Appendix I - pass_scheduler.py"
parent: Appendices
nav_order: 9
permalink: /appendix-Bd/

---

## Appendix I - pass_scheduler.py - Next-Pass Selection

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pass_scheduler.py — Legacy-workflow scheduler for .pass files

This version keeps your original workflow:
  TLE_Set_Builder.py  →  TLE_Parser_Test.py (writes ./Passes/<SAT>_<today>.pass)  →  pass_scheduler.py

What this script does
- Scans the ./Passes directory for all *.pass files.
- Each .pass file format:
    Line 1   : <SATELLITE NAME>
    Line 2+ : <start_utc_iso>,<end_utc_iso>   (e.g., 2025-10-20T03:10:32+00:00,2025-10-20T03:22:45+00:00)
- Finds the FIRST upcoming pass after the given time (default: now, UTC).
- Prints a human-friendly summary AND a machine-readable line, e.g.:
    NEXT_PASS,TELEOS-2,2025-10-20T03:10:32+00:00,2025-10-20T03:22:45+00:00,733

CLI examples
  # Default: pick the next pass from ./Passes using current UTC time
  python pass_scheduler.py

  # Point to a custom directory and also print Singapore local times
  python pass_scheduler.py --dir Passes --local

  # Show all future passes (across all files) starting from now
  python pass_scheduler.py --all --local
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import sys
from pathlib import Path
from typing import Optional, Tuple, List

# Optional import: only used to accept Skyfield Time objects for `current_time`
try:
    from skyfield.api import load as _sf_load  # noqa: F401
except Exception:  # pragma: no cover
    _sf_load = None

ASIA_SG = dt.timezone(dt.timedelta(hours=8))
UTC = dt.timezone.utc

# ----------------------------- helpers -----------------------------

def _ensure_utc_datetime(current_time) -> dt.datetime:
    """Accepts Python datetime or Skyfield Time; returns timezone-aware UTC datetime."""
    # Skyfield Time has .utc_datetime() method
    if hasattr(current_time, "utc_datetime"):
        current_time = current_time.utc_datetime()
    if not isinstance(current_time, dt.datetime):
        raise TypeError("current_time must be datetime or Skyfield Time")
    if current_time.tzinfo is None:
        current_time = current_time.replace(tzinfo=UTC)
    return current_time.astimezone(UTC)

def _iso_utc(d: dt.datetime) -> str:
    if d.tzinfo is None:
        d = d.replace(tzinfo=UTC)
    return d.astimezone(UTC).isoformat()

def _iso_sgt(d: dt.datetime) -> str:
    if d.tzinfo is None:
        d = d.replace(tzinfo=UTC)
    return d.astimezone(ASIA_SG).isoformat()

def humanize_seconds(total: float) -> str:
    """Convert duration in seconds → 'Xm Ys' or 'Xh Ym Zs'."""
    total = int(total)
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    if h > 0:
        return f"{h}h {m}m {s}s"
    elif m > 0:
        return f"{m}m {s}s"
    else:
        return f"{s}s"

# ----------------------- legacy-compatible API -----------------------

def convert_datetime(str_datetime: str) -> dt.datetime:
    """Parse timestamps written by your .pass generator.

    Accepts strings like:
      - '2025-10-20 03:10:32.123456+00:00'
      - '2025-10-20T03:10:32.123456+00:00'
      - '2025-10-20 03:10:32.123456' (assumed UTC)
      - '2025-10-20T03:10:32' (assumed UTC)
    """
    s = str_datetime.strip()
    s = s.replace('T', ' ')
    s = s.replace('+00:00', '').replace('Z', '')
    fmts = [
        '%Y-%m-%d %H:%M:%S.%f',
        '%Y-%m-%d %H:%M:%S',
    ]
    d = None
    for fmt in fmts:
        try:
            d = dt.datetime.strptime(s, fmt)
            break
        except ValueError:
            pass
    if d is None:
        raise ValueError(f"Could not parse datetime string: {str_datetime}")
    return d.replace(tzinfo=UTC)

def load_passfile(pass_file: str | os.PathLike, current_time) -> Optional[Tuple[str, dt.datetime, dt.datetime]]:
    """Return (satname, start_time_utc, end_time_utc) for the first future pass in the file.
    If no upcoming events in the file, return None.
    """
    now_utc = _ensure_utc_datetime(current_time)

    with open(pass_file, 'r', encoding='utf-8') as f:
        satname = f.readline().strip()  # first line
        for line in f:
            if not line.strip():
                continue
            try:
                start_s, end_s = [x.strip() for x in line.split(',')]
            except ValueError:
                continue
            start_dt = convert_datetime(start_s)
            end_dt = convert_datetime(end_s)
            if start_dt > now_utc:
                return satname, start_dt, end_dt
    return None

def select_pass(pass_directory: str = "Passes", current_time=None):
    """Scan all .pass files and return the next upcoming event across all satellites.

    Returns (start_dt_utc, end_dt_utc, satname, duration_seconds)
    or (None, None, None, None) if nothing upcoming.
    """
    if current_time is None:
        current_time = dt.datetime.now(UTC)
    now_utc = _ensure_utc_datetime(current_time)

    path = Path(pass_directory)
    if not path.exists():
        print(f"[INFO] Pass directory not found: {path}")
        return None, None, None, None

    best = None  # (start, end, name)
    for fname in sorted(path.glob('*.pass')):
        try:
            res = load_passfile(fname, now_utc)
        except Exception as e:
            print(f"[WARN] Skipping {fname}: {e}")
            continue
        if res is None:
            continue
        sat, start, end = res
        if best is None or start < best[0]:
            best = (start, end, sat)

    if best is None:
        print("All stored passes complete! Exiting the program!")
        return None, None, None, None

    start, end, sat = best
    duration = (end - start).total_seconds()
    print(f"Next pass: {sat} at {start.isoformat()} for {humanize_seconds(duration)}")
    return start, end, sat, duration

# ------------------------------ CLI ------------------------------

def _list_all_future(pass_directory: str, current_time, show_local: bool):
    now_utc = _ensure_utc_datetime(current_time)
    path = Path(pass_directory)
    rows: List[Tuple[str, dt.datetime, dt.datetime]] = []
    for fname in sorted(path.glob('*.pass')):
        try:
            with open(fname, 'r', encoding='utf-8') as f:
                satname = f.readline().strip()
                for line in f:
                    if not line.strip():
                        continue
                    try:
                        start_s, end_s = [x.strip() for x in line.split(',')]
                    except ValueError:
                        continue
                    start_dt = convert_datetime(start_s)
                    end_dt = convert_datetime(end_s)
                    if start_dt > now_utc:
                        rows.append((satname, start_dt, end_dt))
        except Exception as e:
            print(f"[WARN] Failed reading {fname}: {e}")
    rows.sort(key=lambda r: r[1])
    if not rows:
        print("[INFO] No upcoming passes found.")
        return
    print("\n=== Upcoming passes ===")
    for sat, s, e in rows:
        dur = int((e - s).total_seconds())
        line = f"{sat:12s}  {s.isoformat()}  →  {e.isoformat()}  ({humanize_seconds(dur)})"
        if show_local:
            line += f"  |  SGT: {_iso_sgt(s)} → {_iso_sgt(e)}"
        print(line)
    if rows:
        sat, s, e = rows[0]
        print("\nNEXT_PASS,{},{},{},{}".format(sat, _iso_utc(s), _iso_utc(e), int((e - s).total_seconds())))

def main():
    ap = argparse.ArgumentParser(description="Select the next satellite pass from legacy .pass files.")
    ap.add_argument('--dir', default='Passes', help='Directory containing .pass files (default: Passes)')
    ap.add_argument('--local', action='store_true', help='Also print Singapore local (SGT) times')
    ap.add_argument('--all', action='store_true', help='List all upcoming passes instead of only the next one')
    ap.add_argument('--now', type=str, default=None,
                    help='Override current UTC time (ISO, e.g., 2025-10-20T03:00:00)')

    args = ap.parse_args()

    # Determine current time
    if args.now:
        t = args.now.replace('T', ' ')
        fmts = ['%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M']
        parsed = None
        for fmt in fmts:
            try:
                parsed = dt.datetime.strptime(t, fmt).replace(tzinfo=UTC)
                break
            except ValueError:
                continue
        if parsed is None:
            print('[ERROR] --now must be ISO like 2025-10-20T03:00:00')
            return 2
        now = parsed
    else:
        now = dt.datetime.now(UTC)

    if args.all:
        _list_all_future(args.dir, now, args.local)
        return 0

    start, end, sat, dur = select_pass(args.dir, now)
    if start is None:
        return 0

    # Human-friendly
    print("\n==== NEXT PASS ====")
    print(f"Satellite : {sat}")
    print(f"Start UTC : {start.isoformat()}")
    print(f"End   UTC : {end.isoformat()}")
    print(f"Duration  : {humanize_seconds(dur)}")
    if args.local:
        print(f"Start SGT : {_iso_sgt(start)}")
        print(f"End   SGT : {_iso_sgt(end)}")

    # Machine-readable for scripts
    print(f"NEXT_PASS,{sat},{_iso_utc(start)},{_iso_utc(end)},{int(dur)}")
    return 0

if __name__ == '__main__':
    raise SystemExit(main())

```