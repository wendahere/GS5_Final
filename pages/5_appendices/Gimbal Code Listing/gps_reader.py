# gps_reader.py — u-blox quickstart (UBX first, NMEA fallback) + delayed heading catch
# deps: pip install pyserial pyubx2

COM = "COM5"  # <<< CHANGE THIS VALUE HERE >>>

from serial import Serial
from serial.tools import list_ports
from datetime import datetime, timezone, timedelta
from pyubx2 import UBXReader, UBXMessage

# ---------- helpers ----------
def _auto_deg(v):
    if v is None:
        return None
    try:
        return (v * 1e-7) if abs(v) > 180 else float(v)
    except Exception:
        return None

def _auto_alt_m(h):
    if h is None:
        return None
    try:
        return (h / 1000.0) if abs(h) > 1000 else float(h)
    except Exception:
        return None

def _auto_heading_deg(rh, ah):
    def scale(x):
        if x is None:
            return None
        return (x * 1e-5) if abs(x) > 400 else float(x)
    h = scale(rh)
    a = scale(ah)
    if h is not None:
        h = (h - 90.0) % 360.0
    return h, a

def _dm_to_deg(dm: str, hemi: str):
    if not dm or "." not in dm:
        return None
    i = dm.find(".")
    deg = float(dm[:i-2]) if i >= 2 else 0.0
    mins = float(dm[i-2:])
    val = deg + mins / 60.0
    return -val if hemi in ("S", "W") else val

# ---------- public API ----------
def read_once(port=COM, baud=38400, max_wait_s=8.0, want_heading=True, extra_heading_wait_s=2.0):
    """
    Get one fix: returns {time_utc, lat, lon, alt_m, heading_deg, heading_acc_deg, source}.
    - Prefers UBX NAV-PVT; falls back to NMEA (GGA/GLL).
    - After position is available, keeps listening up to extra_heading_wait_s for NAV-RELPOSNED.
    """
    with Serial(port, baud, timeout=0.5) as s:
        ubr = UBXReader(s, protfilter=7)  # UBX+NMEA
        deadline = datetime.now(timezone.utc) + timedelta(seconds=max_wait_s)

        time_utc = lat = lon = alt_m = None
        heading_deg = heading_acc_deg = None
        source = None

        # 1) Acquire position (UBX preferred)
        while datetime.now(timezone.utc) < deadline:
            raw, msg = ubr.read()

            if isinstance(msg, UBXMessage):
                if msg.identity == "NAV-PVT" and getattr(msg, "fixType", 0) >= 2:
                    if getattr(msg, "validTime", 0):
                        time_utc = datetime(
                            msg.year, msg.month, msg.day,
                            msg.hour, msg.min, msg.second,
                            tzinfo=timezone.utc
                        ) + timedelta(microseconds=int(getattr(msg, "nano", 0)/1000))
                    lat = _auto_deg(getattr(msg, "lat", None))
                    lon = _auto_deg(getattr(msg, "lon", None))
                    alt_m = _auto_alt_m(getattr(msg, "height", None))
                    if (lat is not None) and (lon is not None):
                        source = "UBX"
                        break  # got position; move on to (optional) heading window

                elif want_heading and msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                    rh = getattr(msg, "relPosHeading", None)
                    ah = getattr(msg, "accHeading", None)
                    heading_deg, heading_acc_deg = _auto_heading_deg(rh, ah)

            # NMEA fallback
            if (lat is None or lon is None) and raw:
                try:
                    line = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    line = ""
                if line.startswith("$GNGGA") or line.startswith("$GPGGA") or line.startswith("$GAGGA"):
                    parts = line.split(",")
                    if len(parts) >= 10:
                        fixq = int(parts[6]) if parts[6].isdigit() else 0
                        if fixq >= 1:
                            lat = _dm_to_deg(parts[2], parts[3])
                            lon = _dm_to_deg(parts[4], parts[5])
                            alt_m = float(parts[9]) if parts[9] else None
                            hhmmss = parts[1]
                            if len(hhmmss) >= 6:
                                h = int(hhmmss[0:2]); m = int(hhmmss[2:4]); sec = float(hhmmss[4:])
                                us = int((sec - int(sec)) * 1_000_000)
                                now = datetime.now(timezone.utc)
                                time_utc = now.replace(hour=h, minute=m, second=int(sec), microsecond=us)
                            source = "NMEA-GGA"
                            break
                if line.startswith("$GNGLL") or line.startswith("$GPGLL") or line.startswith("$GAGLL"):
                    parts = line.split(",")
                    if len(parts) >= 7 and parts[6] in ("A", "D"):
                        lat = _dm_to_deg(parts[1], parts[2])
                        lon = _dm_to_deg(parts[3], parts[4])
                        hhmmss = parts[5]
                        if len(hhmmss) >= 6:
                            h = int(hhmmss[0:2]); m = int(hhmmss[2:4]); sec = float(hhmmss[4:])
                            us = int((sec - int(sec)) * 1_000_000)
                            now = datetime.now(timezone.utc)
                            time_utc = now.replace(hour=h, minute=m, second=int(sec), microsecond=us)
                        source = "NMEA-GLL"
                        break

        if (lat is None) or (lon is None):
            raise TimeoutError("No position within timeout; increase max_wait_s or check sky view.")

        # 2) Short window to catch heading (NAV-RELPOSNED)
        if want_heading and heading_deg is None and extra_heading_wait_s > 0:
            heading_deadline = datetime.now(timezone.utc) + timedelta(seconds=extra_heading_wait_s)
            while datetime.now(timezone.utc) < heading_deadline:
                _, msg = ubr.read()
                if isinstance(msg, UBXMessage) and msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                    rh = getattr(msg, "relPosHeading", None)
                    ah = getattr(msg, "accHeading", None)
                    heading_deg, heading_acc_deg = _auto_heading_deg(rh, ah)
                    break

        return {
            "time_utc": time_utc,
            "lat": lat, "lon": lon, "alt_m": alt_m,
            "heading_deg": heading_deg, "heading_acc_deg": heading_acc_deg,
            "source": source,
        }

def stream_status(port=COM, baud=38400, timeout=1.0):
    FIXTYPE_MAP = {0:"No fix",1:"DR only",2:"2D",3:"3D",4:"GNSS+DR",5:"Time only"}
    with Serial(port, baud, timeout=timeout) as s:
        ubr = UBXReader(s, protfilter=1)  # UBX
        heading = heading_acc = None
        print(f"[GPS] Listening on {port}@{baud} (Ctrl+C to stop)")
        while True:
            _, msg = ubr.read()
            if not isinstance(msg, UBXMessage):
                continue
            if msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                heading, heading_acc = _auto_heading_deg(getattr(msg, "relPosHeading", None),
                                                         getattr(msg, "accHeading", None))
                continue
            if msg.identity == "NAV-PVT":
                t = None
                if getattr(msg, "validTime", 0):
                    t = datetime(msg.year, msg.month, msg.day, msg.hour, msg.min, msg.second,
                                 tzinfo=timezone.utc) + timedelta(microseconds=int(getattr(msg, "nano", 0)/1000))
                fixType = getattr(msg, "fixType", 0)
                numSV = getattr(msg, "numSV", None)
                lat = _auto_deg(getattr(msg, "lat", None))
                lon = _auto_deg(getattr(msg, "lon", None))
                alt_m = _auto_alt_m(getattr(msg, "height", None))
                line = f"[{t.isoformat() if t else 'no-time'}] fix={FIXTYPE_MAP.get(fixType, fixType)}"
                if numSV is not None:
                    line += f", sats={numSV}"
                if fixType >= 2 and (lat is not None) and (lon is not None):
                    line += f", lat={lat:.7f}, lon={lon:.7f}"
                    if alt_m is not None:
                        line += f", alt={alt_m:.2f} m"
                if heading is not None:
                    line += f", heading={heading:.2f}° ±{(heading_acc or 0.0):.2f}°"
                print(line)

# ---------- run directly ----------
if __name__ == "__main__":
    print("Available ports:")
    for p in list(list_ports.comports()):
        print(f" - {p.device} | {p.description}")

    BAUD = 38400

    try:
        fix = read_once(port=COM, baud=BAUD, max_wait_s=8, want_heading=True, extra_heading_wait_s=2.0)
        print("\n--- GPS FIX ---")
        print("UTC Time :", fix["time_utc"])
        print(f"Latitude : {fix['lat']:.7f}")
        print(f"Longitude: {fix['lon']:.7f}")
        if fix["alt_m"] is not None:
            print(f"Altitude : {fix['alt_m']:.2f} m")
        if fix["heading_deg"] is not None:
            acc = fix["heading_acc_deg"] or 0.0
            print(f"Heading  : {fix['heading_deg']:.2f}° (±{acc:.2f}°)")
        else:
            print("Heading  : (no RELPOSNED yet)")
        print("Source   :", fix["source"])
        print("----------------\n")

        # Uncomment to watch in real-time:
        # stream_status(port=COM, baud=BAUD, timeout=1.0)

    except TimeoutError as e:
        print("[ERROR]", e)
    except KeyboardInterrupt:
        print("\n[GPS] Stopped.")
