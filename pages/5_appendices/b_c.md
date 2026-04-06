---
title: "Appendix H - live_with_gps.py"
parent: Appendices
nav_order: 8
permalink: /appendix-bc/

---

## Appendix H - live_with_gps.py - Real-Time Tracking Loop

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_tracker.py — real-time TLE tracker using PCHIP Hermite + smart flip (hysteresis)

Remounted gimbal coordinate system:
• AZ: -90° (West) → 0° (North) → +90° (East)
• EL: -90° (North horizon) → 0° (Up) → +90° (South horizon)

Always uses gimbal_lib.GimbalController (no direct Galil path).
"""

import time
import datetime
from pathlib import Path
from typing import Tuple, List
from skyfield.api import load, wgs84, utc  # <-- utc imported (not strictly needed with ts.now())
from gps_reader import read_once

# =========================
# -------- SETTINGS -------
# =========================
SAT_NAME        = "TELEOS-2"

# Ground station (hardcoded; GPS block kept but disabled below)
#GS_LAT          = 1.2966
#GS_LON          = 103.7764
#GS_ALT_M        = 30.0

    # -------------------------
    # GPS: get live GS position
    # -------------------------
print("[GPS] Waiting for valid fix...")
fix = read_once(port="COM5", baud=38400, max_wait_s=8, want_heading=True, extra_heading_wait_s=2.0)
if (fix["lat"] is None) or (fix["lon"] is None):
    raise RuntimeError("[GPS] No valid GPS position fix — cannot continue.")
GS_LAT, GS_LON, GS_ALT_M = fix["lat"], fix["lon"], (fix["alt_m"] or 0.0)
print(f"[GPS] GS set to {GS_LAT:.7f}, {GS_LON:.7f}, {GS_ALT_M:.2f} m (src={fix['source']})")


UPDATE_INTERVAL = 2.0        # seconds between major updates (knot spacing)
INTERP_STEPS    = 6          # sub-steps within UPDATE_INTERVAL (sends per UPDATE_INTERVAL)
ELEV_CUTOFF_DEG = 0.0       # ignore targets below true horizon (sky frame)

# Controller (always via gimbal_lib)
ADDRESS         = "192.168.1.2 --direct"
DRY_RUN         = False      # True = no hardware I/O

# Frame/limits
AZ_ZERO_DEG     = 0.0        # 0° = North
FENCE_MARGIN    = 1.0        # deg soft margin from hard stops
AZ_MIN, AZ_MAX  = -90.0,  90.0
EL_MIN, EL_MAX  = -90.0,  90.0

# Flip hysteresis (prevents chatter near ±90° from AZ_ZERO)
FLIP_ON_DEG     = 92.0
FLIP_OFF_DEG    = 88.0

# =========================
# -------- HELPERS --------
# =========================
def wrap_pm180(deg0_360: float) -> float:
    return ((deg0_360 + 180.0) % 360.0) - 180.0

def rewrap_0_360(a: float) -> float:
    return a % 360.0

def clamp_with_fence(val: float, vmin: float, vmax: float, fence: float) -> float:
    return max(vmin + fence, min(vmax - fence, val))

def find_sat(sat_list, name: str):
    u = name.strip().upper()
    for s in sat_list:
        if s.name.strip().upper() == u:
            return s
    for s in sat_list:
        if u in s.name.strip().upper():
            return s
    return None

def unwrap_shortest(seq_deg: List[float], ref: float) -> List[float]:
    """Unwrap azimuth (deg) sequence to avoid 0/360 jumps near ref."""
    out = []
    prev = ref
    for a in seq_deg:
        k = round((prev - a) / 360.0)
        a_unw = a + 360.0 * k
        if abs(a_unw - prev) > 180.0:
            a_unw += -360.0 if a_unw > prev else 360.0
        out.append(a_unw)
        prev = a_unw
    return out

def pchip_slopes(x: List[float], y: List[float]) -> List[float]:
    """Fritsch–Carlson monotone cubic slopes."""
    n = len(x)
    assert n == len(y) and n >= 2
    m = [0.0] * n
    d = [(y[i+1] - y[i]) / (x[i+1] - x[i]) for i in range(n-1)]
    m[0] = d[0]
    m[-1] = d[-1]
    for i in range(1, n-1):
        if d[i-1] * d[i] <= 0.0:
            m[i] = 0.0
        else:
            w1 = 2.0 * (x[i+1] - x[i]) + (x[i] - x[i-1])
            w2 = (x[i+1] - x[i]) + 2.0 * (x[i] - x[i-1])
            m[i] = (w1 + w2) / (w1 / d[i-1] + w2 / d[i])
    return m

def hermite_eval(x: List[float], y: List[float], m: List[float], xq: float) -> float:
    """Evaluate cubic Hermite at xq (within x[0]..x[-1])."""
    k = 0
    for i in range(len(x) - 1):
        if x[i] <= xq <= x[i+1]:
            k = i
            break
    h = x[k+1] - x[k]
    t = (xq - x[k]) / h
    h00 = (1 + 2*t) * (1 - t)**2
    h10 = t * (1 - t)**2
    h01 = t**2 * (3 - 2*t)
    h11 = t**2 * (t - 1)
    return (h00 * y[k] +
            h10 * h * m[k] +
            h01 * y[k+1] +
            h11 * h * m[k+1])

def decide_flip_with_hysteresis(az_rel: float, flipped_state: bool) -> bool:
    """Hysteresis around ±90° from AZ_ZERO."""
    a = abs(az_rel)
    if not flipped_state and a > FLIP_ON_DEG:
        return True
    if flipped_state and a < FLIP_OFF_DEG:
        return False
    return flipped_state

def map_to_gimbal_frame_with_state(az_sky: float, el_sky: float, az_zero: float, flipped_state: bool) -> Tuple[float, float, bool]:
    """
    Convert sky AZ/EL to gimbal frame with hysteretic flip decision.
    Returns (AZ_gimbal, EL_gimbal, flipped_state_new).
    """
    az_rel = wrap_pm180(az_sky - az_zero)
    flipped_new = decide_flip_with_hysteresis(az_rel, flipped_state)

    if flipped_new:
        # mirror AZ across 180 by shifting ±180 to bring into [-90, +90]
        if az_rel > 0:
            az_rel -= 180.0
        else:
            az_rel += 180.0
        el_gim = 90.0 - el_sky  # flip across zenith
    else:
        el_gim = el_sky - 90.0  # native mapping

    return az_rel, el_gim, flipped_new

# =========================
# ---------- MAIN ---------
# =========================
def main():
    # Load TLEs
    today = datetime.date.today()
    tle_path = Path(__file__).parent / "TLE" / f"TLE{today}.tle"
    if not tle_path.exists():
        raise FileNotFoundError(f"TLE file not found: {tle_path}")

    sats = load.tle_file(str(tle_path))
    sat = find_sat(sats, SAT_NAME)
    if sat is None:
        raise RuntimeError(f"Satellite '{SAT_NAME}' not found in {tle_path.name}")
    print(f"[INFO] Tracking: {sat.name}")

    # Time + site
    ts = load.timescale()
    gs = wgs84.latlon(GS_LAT, GS_LON, elevation_m=GS_ALT_M)

    # Controller
    gimbal = None
    if not DRY_RUN:
        from gimbal_lib import GimbalController  # always via your library
        gimbal = GimbalController(ADDRESS)
        print("[GIMBAL] Connected.")

    print(f"[CFG] GS=({GS_LAT:.6f},{GS_LON:.6f},{GS_ALT_M:.1f}m)  step={UPDATE_INTERVAL}s  "
          f"interp={INTERP_STEPS}  mode=PCHIP  AZ_zero={AZ_ZERO_DEG}°  dry={DRY_RUN}")

    flipped_state = False  # persistent hysteresis state

    try:
        while True:
            # Use timezone-aware times from Skyfield itself
            t0 = ts.now()
            # Build tmid and t1 using timezone-aware datetimes from t0
            t0_dt = t0.utc_datetime().replace(tzinfo=utc)  # ensure tz-aware
            tmid = ts.utc(t0_dt + datetime.timedelta(seconds=0.5 * UPDATE_INTERVAL))
            t1   = ts.utc(t0_dt + datetime.timedelta(seconds=UPDATE_INTERVAL))

            # Sample at knots
            alt0, az0, _ = (sat - gs).at(t0).altaz()
            altm, azm, _ = (sat - gs).at(tmid).altaz()
            alt1, az1, _ = (sat - gs).at(t1).altaz()

            # Tiny knot vector (seconds from t0)
            X = [0.0, 0.5 * UPDATE_INTERVAL, UPDATE_INTERVAL]

            # Elevation: no wrap
            Y_el = [alt0.degrees, altm.degrees, alt1.degrees]
            M_el = pchip_slopes(X, Y_el)

            # Azimuth: unwrap around az0 to avoid 0/360 jumps
            Y_az_raw = [az0.degrees, azm.degrees, az1.degrees]
            Y_az_unw = unwrap_shortest(Y_az_raw, ref=az0.degrees)
            M_az = pchip_slopes(X, Y_az_unw)

            # Sub-steps within [t0, t1]
            dt_sub = UPDATE_INTERVAL / INTERP_STEPS
            for i in range(INTERP_STEPS + 1):
                xq = i * dt_sub

                el_sky = hermite_eval(X, Y_el, M_el, xq)
                az_sky_unw = hermite_eval(X, Y_az_unw, M_az, xq)
                az_sky = rewrap_0_360(az_sky_unw)

                # Apply elevation cutoff before mapping to gimbal frame
                if el_sky < ELEV_CUTOFF_DEG:
                    time.sleep(dt_sub)
                    continue

                az_gim, el_gim, flipped_state = map_to_gimbal_frame_with_state(
                    az_sky, el_sky, AZ_ZERO_DEG, flipped_state
                )

                # Clamp inside gimbal limits with fence
                az_gim = clamp_with_fence(az_gim, AZ_MIN, AZ_MAX, FENCE_MARGIN)
                el_gim = clamp_with_fence(el_gim, EL_MIN, EL_MAX, FENCE_MARGIN)

                if DRY_RUN:
                    msg = (f"[GO] AZ_gim={az_gim:7.2f}°  EL_gim={el_gim:7.2f}°  "
                           f"(sky AZ={az_sky:7.2f}°, EL={el_sky:7.2f}°)")
                    if flipped_state: msg += " [FLIPPED]"
                    print(msg)
                else:
                    try:
                        gimbal.degSteer(az_gim, el_gim, absolute=True, wait=False)
                    except Exception as e:
                        print(f"[WARN] Steering command failed (non-blocking). Continuing. Details: {e}")

                time.sleep(dt_sub)

    except KeyboardInterrupt:
        print("\n[STOP] Tracking stopped by user.")
    finally:
        if gimbal is not None:
            gimbal.close()

if __name__ == "__main__":
    main()

```