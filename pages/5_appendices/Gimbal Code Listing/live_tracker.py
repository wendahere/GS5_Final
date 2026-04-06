#!/usr/bin/env python3
"""
live_tracker.py — simple real-time TLE tracker (interpolated + smart flip)

Your remounted gimbal coordinate system:
• AZ: -90° (West) → 0° (North) → +90° (East)
• EL: -90° (North horizon) → 0° (Up) → +90° (South horizon)
"""

import time
import datetime
from pathlib import Path
from skyfield.api import load, wgs84

# -------- USER SETTINGS --------
SAT_NAME        = "ASIASAT 5"

# Hardcoded Ground Station (GPS block kept below)
GS_LAT          = 1.2966
GS_LON          = 103.7764
GS_ALT_M        = 30

UPDATE_INTERVAL = 2.0      # seconds between Skyfield updates
INTERP_STEPS    = 5        # interpolation steps for smoothness
ELEV_CUTOFF_DEG = 0.0      # ignore below horizon

ADDRESS         = "192.168.1.2 --direct"
AZ_ZERO_DEG     = 0.0      # 0° = North; set 180.0 if you home to South
FENCE_MARGIN    = 1.0      # deg
DRY_RUN         = True     # True = no hardware I/O
# --------------------------------

### GPS (optional, disabled for now)
"""
from gps_reader import read_once
try:
    fix = read_once(port="COM6", baud=38400, max_wait_s=8)
    GS_LAT, GS_LON, GS_ALT_M = fix["lat"], fix["lon"], fix["alt_m"] or 0.0
    print(f"[GPS] GS set to {GS_LAT:.7f}, {GS_LON:.7f}, {GS_ALT_M:.2f} m (src={fix['source']})")
except TimeoutError as e:
    print("[GPS] No fix:", e, "— using hardcoded GS.")
"""
# --------------------------------

# Limits (±90° on both axes)
AZ_MIN, AZ_MAX = -90.0, 90.0
EL_MIN, EL_MAX = -90.0, 90.0

# ---- Helpers ----
def wrap_pm180(deg0_360: float) -> float:
    return ((deg0_360 + 180.0) % 360.0) - 180.0

def clamp_with_fence(val: float, vmin: float, vmax: float, fence: float) -> float:
    return max(vmin + fence, min(vmax - fence, val))

def interp_az_shortest(az0_deg: float, az1_deg: float, f: float) -> float:
    delta = ((az1_deg - az0_deg + 180.0) % 360.0) - 180.0
    return (az0_deg + f * delta) % 360.0

def find_sat(sat_list, name: str):
    u = name.strip().upper()
    for s in sat_list:
        if s.name.strip().upper() == u:
            return s
    for s in sat_list:
        if u in s.name.strip().upper():
            return s
    return None

def map_to_gimbal_frame(az_sky: float, el_sky: float, az_zero: float):
    """
    Convert sky AZ/EL to gimbal frame with smart flip.
    Returns (AZ_gimbal, EL_gimbal, flipped_flag).
    """
    az_rel = wrap_pm180(az_sky - az_zero)
    flipped = False

    # If target is behind ±90° AZ, mirror AZ across 180° and flip EL across zenith
    if az_rel > 90.0:
        az_rel -= 180.0
        flipped = True
    elif az_rel < -90.0:
        az_rel += 180.0
        flipped = True

    # EL mapping:
    # - Not flipped:  EL_gim = EL_sky - 90  (0=Up in gimbal frame)
    # - Flipped:      EL_gim = 90 - EL_sky  (sign reverses across zenith)
    el_gim = (90.0 - el_sky) if flipped else (el_sky - 90.0)
    return az_rel, el_gim, flipped

# ---- Main ----
def main():
    today = datetime.date.today()
    tle_path = Path(__file__).parent / "TLE" / f"TLE{today}.tle"
    if not tle_path.exists():
        raise FileNotFoundError(f"TLE file not found: {tle_path}")

    sats = load.tle_file(str(tle_path))
    sat = find_sat(sats, SAT_NAME)
    if sat is None:
        raise RuntimeError(f"Satellite '{SAT_NAME}' not found in {tle_path.name}")
    print(f"[INFO] Tracking: {sat.name}")

    ts = load.timescale()
    gs = wgs84.latlon(GS_LAT, GS_LON, elevation_m=GS_ALT_M)

    gimbal = None
    if not DRY_RUN:
        from gimbal_lib import GimbalController
        gimbal = GimbalController(ADDRESS)
        print("[GIMBAL] Connected.")

    print(f"[CFG] GS=({GS_LAT:.6f},{GS_LON:.6f},{GS_ALT_M:.1f}m)  step={UPDATE_INTERVAL}s  interp={INTERP_STEPS}  AZ_zero={AZ_ZERO_DEG}°  dry={DRY_RUN}")

    try:
        while True:
            t0 = ts.now()
            t1 = ts.utc(t0.utc_datetime() + datetime.timedelta(seconds=UPDATE_INTERVAL))
            alt0, az0, _ = (sat - gs).at(t0).altaz()
            alt1, az1, _ = (sat - gs).at(t1).altaz()

            for i in range(INTERP_STEPS + 1):
                f = i / INTERP_STEPS
                az_sky = interp_az_shortest(az0.degrees, az1.degrees, f)  # 0..360
                el_sky = alt0.degrees + f * (alt1.degrees - alt0.degrees) # 0..90

                if el_sky < ELEV_CUTOFF_DEG:
                    time.sleep(UPDATE_INTERVAL / INTERP_STEPS)
                    continue

                # Map to gimbal frame (with flip when needed)
                az_gim, el_gim, flipped = map_to_gimbal_frame(az_sky, el_sky, AZ_ZERO_DEG)

                # Clamp inside limits
                az_gim = clamp_with_fence(az_gim, AZ_MIN, AZ_MAX, FENCE_MARGIN)
                el_gim = clamp_with_fence(el_gim, EL_MIN, EL_MAX, FENCE_MARGIN)

                if DRY_RUN:
                    msg = (f"[GO] AZ_gim={az_gim:7.2f}°  EL_gim={el_gim:7.2f}°  "
                           f"(sky AZ={az_sky:7.2f}°, EL={el_sky:7.2f}°)")
                    if flipped:
                        msg += " [FLIPPED]"
                    print(msg)
                else:
                    # Send gimbal-frame angles directly
                    gimbal.degSteer(az_gim, el_gim, absolute=True, wait=True)  # <— changed wait to True

                time.sleep(UPDATE_INTERVAL / INTERP_STEPS)

    except KeyboardInterrupt:
        print("\n[STOP] Tracking stopped by user.")
    finally:
        if gimbal is not None:
            gimbal.close()

if __name__ == "__main__":
    main()
