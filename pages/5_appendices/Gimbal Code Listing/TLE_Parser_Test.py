#!/usr/bin/env python3
"""
TLE_Parser_Test.py — Auto-generates .pass file + prints SKY/GIMBAL orientation

Behavior:
• Always prints current satellite sky position + gimbal angles.
• Always generates an updated ./Passes/<SAT>_<today>.pass file
  for use by pass_scheduler.py.
"""

import argparse
import datetime as dt
from pathlib import Path
from skyfield.api import load, wgs84
import math

# ---------------- USER SETTINGS (defaults) ----------------
SAT_NAME = "TELEOS-2"
GS_LAT   = 1.2966
GS_LON   = 103.7764
GS_ALT_M = 30.0
DEFAULT_CUTOFF_EL   = 0.0    # deg
DEFAULT_WINDOW_H    = 24      # hours
DEFAULT_PASS_DIR    = "Passes"
# -----------------------------------------------------------

def _wrap180(deg):
    return ((deg + 180.0) % 360.0) - 180.0

def _utc_iso(ts: dt.datetime) -> str:
    """Return ISO string with explicit UTC offset +00:00."""
    if ts.tzinfo is None:
        ts = ts.replace(tzinfo=dt.timezone.utc)
    return ts.astimezone(dt.timezone.utc).isoformat()

def _slant_range_km_at(time_sf, sat, loc) -> float:
    delta = sat - loc
    p = delta.at(time_sf).position.km
    return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)

def _enumerate_triplets(sat, loc, t0_sf, t1_sf, cutoff_el_deg):
    """Yield (rise_t, max_t, set_t) Skyfield times for each pass above cutoff_el_deg."""
    t, events = sat.find_events(loc, t0_sf, t1_sf, altitude_degrees=cutoff_el_deg)
    i = 0
    while i + 2 < len(events):
        if events[i] == 0 and events[i+1] == 1 and events[i+2] == 2:
            yield (t[i], t[i+1], t[i+2])
            i += 3
        else:
            i += 1

def generate_pass_list(sat, gs_llh, hours_ahead=DEFAULT_WINDOW_H,
                       cutoff_el_deg=DEFAULT_CUTOFF_EL,
                       cutoff_range_km=None,
                       out_dir=DEFAULT_PASS_DIR):
    """Write legacy .pass file with all passes in next N hours."""
    lat, lon, alt_m = gs_llh
    loc = wgs84.latlon(lat, lon, elevation_m=alt_m)
    ts = load.timescale()
    now_sf = ts.now()
    t0_sf = now_sf
    t1_sf = now_sf + hours_ahead / 24.0

    Path(out_dir).mkdir(parents=True, exist_ok=True)
    pass_path = Path(out_dir) / f"{sat.name}_{dt.date.today()}.pass"

    lines = [sat.name + "\n"]
    count = 0
    for rise_t, max_t, set_t in _enumerate_triplets(sat, loc, t0_sf, t1_sf, cutoff_el_deg):
        # Optional zenith-range filter
        if cutoff_range_km is not None:
            rng = _slant_range_km_at(max_t, sat, loc)
            if rng > float(cutoff_range_km):
                continue

        start_dt = rise_t.utc_datetime().replace(tzinfo=dt.timezone.utc)
        end_dt   = set_t.utc_datetime().replace(tzinfo=dt.timezone.utc)
        if end_dt <= dt.datetime.now(dt.timezone.utc):
            continue  # skip past events

        lines.append(f"{_utc_iso(start_dt)},{_utc_iso(end_dt)}\n")
        count += 1

    pass_path.write_text(''.join(lines), encoding='utf-8')
    if count > 0:
        print(f"[WRITE] {pass_path} ({count} pass(es))")
    else:
        print(f"[WARN] No visible passes found in next {hours_ahead} hours (cutoff {cutoff_el_deg}°).")
    return count

def print_now_pose(sat, gs_llh):
    """Print current satellite sky position + gimbal orientation."""
    lat, lon, alt_m = gs_llh
    ts = load.timescale()
    gs = wgs84.latlon(lat, lon, elevation_m=alt_m)
    alt, az, _ = (sat - gs).at(ts.now()).altaz()

    az_sky = az.degrees
    el_sky = alt.degrees

    # Map to gimbal frame with flip logic
    el_gimbal = el_sky - 90.0
    az_wrapped = _wrap180(az_sky)
    flipped = False
    if -90.0 <= az_wrapped <= 90.0:
        az_gim = az_wrapped
        el_gim = el_gimbal
    else:
        flipped = True
        az_gim = _wrap180(az_wrapped - 180.0) if az_wrapped > 90.0 else _wrap180(az_wrapped + 180.0)
        el_gim = -el_gimbal

    print(f"SKY:     AZ={az_sky:.2f}°, EL={el_sky:.2f}°")
    tag = " [FLIPPED]" if flipped else ""
    print(f"GIMBAL:  AZ={az_gim:.2f}°, EL={el_gim:.2f}°{tag}")

def main():
    ap = argparse.ArgumentParser(description="Auto-generate .pass file + show SKY/GIMBAL angles.")
    ap.add_argument("--sat", type=str, default=SAT_NAME, help="Satellite name exactly as in today's TLE file.")
    ap.add_argument("--gs-lat", type=float, default=GS_LAT)
    ap.add_argument("--gs-lon", type=float, default=GS_LON)
    ap.add_argument("--gs-alt", type=float, default=GS_ALT_M)
    ap.add_argument("--cutoff-el", type=float, default=DEFAULT_CUTOFF_EL, help="Elevation cutoff (deg) for passes.")
    ap.add_argument("--cutoff-range", type=float, default=None, help="Optional max zenith slant range (km).")
    ap.add_argument("--hours", type=int, default=DEFAULT_WINDOW_H, help="Look-ahead window (hours).")
    args = ap.parse_args()

    # Load today's TLE
    tle_dir = Path(__file__).parent / "TLE"
    tle_path = tle_dir / f"TLE{dt.date.today()}.tle"
    if not tle_path.exists():
        raise FileNotFoundError(f"TLE file not found: {tle_path}")

    sats = load.tle_file(str(tle_path))
    sat = next((s for s in sats if s.name.strip().upper() == args.sat.strip().upper()), None)
    if sat is None:
        print(f"[ERROR] '{args.sat}' not in {tle_path.name}")
        return 2

    gs_llh = (args.gs_lat, args.gs_lon, args.gs_alt)

    # 1️⃣ Print current sky/gimbal angles
    print_now_pose(sat, gs_llh)

    # 2️⃣ Always regenerate pass file for Pass_Scheduler
    generate_pass_list(
        sat, gs_llh,
        hours_ahead=args.hours,
        cutoff_el_deg=args.cutoff_el,
        cutoff_range_km=args.cutoff_range,
        out_dir=DEFAULT_PASS_DIR
    )

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
