# gps_satellite_02102026.py — u-blox GNSS reader: position + heading + satellites
# deps: pip install pyserial pyubx2

from serial import Serial
from serial.tools import list_ports
from datetime import datetime, timezone, timedelta
from pyubx2 import UBXReader, UBXMessage

GNSS_MAP = {
    0: "GPS",
    1: "SBAS",
    2: "Galileo",
    3: "BeiDou",
    4: "IMES",
    5: "QZSS",
    6: "GLONASS",
}

FIXTYPE_MAP = {
    0: "No fix",
    1: "DR only",
    2: "2D",
    3: "3D",
    4: "GNSS+DR",
    5: "Time only",
}

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
        try:
            return (x * 1e-5) if abs(x) > 400 else float(x)
        except Exception:
            return None

    heading = scale(rh)
    acc = scale(ah)
    if heading is not None:
        heading = (heading - 90.0) % 360.0
    return heading, acc

def _fmt_sat(s):
    gnss = GNSS_MAP.get(getattr(s, "gnssId", None), str(getattr(s, "gnssId", "?")))
    sv   = getattr(s, "svId", "?")
    cno  = getattr(s, "cno", None)
    elev = getattr(s, "elev", None)
    azim = getattr(s, "azim", None)
    return f"{gnss:7} SV{sv:>3}  C/N0={str(cno):>2} dBHz  elev={str(elev):>3}°  az={str(azim):>3}°"

def stream_status(port="COM6", baud=38400, timeout=1.0, sat_print_every_s=2.0, sat_top_n=12):
    """
    Live print:
      - NAV-PVT: time/position/fix/sats_used
      - NAV-RELPOSNED: heading + acc (if valid)
      - NAV-SAT: satellites in view (top N by C/N0)

    Compatible with pyubx2 versions that DON'T have UBXReader.ERR_IGNORE
    because we just catch exceptions from ubr.read().
    """
    with Serial(port, baud, timeout=timeout) as s:
        # Accept UBX + NMEA + RTCM (we'll ignore RTCM frames)
        ubr = UBXReader(s, protfilter=7)

        heading = heading_acc = None
        last_sat_print = datetime.now(timezone.utc)

        print(f"\n[GPS] Listening on {port}@{baud} (Ctrl+C to stop)")
        print("-" * 70)

        while True:
            try:
                raw, msg = ubr.read()
            except Exception:
                # Older pyubx2 raises on unknown headers (e.g. RTCM). Ignore and continue.
                continue

            # Skip RTCM frames (RTCM3 starts with 0xD3)
            if raw and raw[:1] == b"\xD3":
                continue

            if not isinstance(msg, UBXMessage):
                continue

            # Satellites list (NAV-SAT)
            if msg.identity == "NAV-SAT":
                now = datetime.now(timezone.utc)
                if now - last_sat_print >= timedelta(seconds=sat_print_every_s):
                    last_sat_print = now
                    sats = getattr(msg, "sats", []) or []
                    if sats:
                        print("\n[SAT] Satellites in view (top by C/N0):")
                        sats_sorted = sorted(sats, key=lambda x: getattr(x, "cno", 0) or 0, reverse=True)
                        for srec in sats_sorted[:sat_top_n]:
                            print(" ", _fmt_sat(srec))
                    else:
                        print("\n[SAT] NAV-SAT received but no sats list parsed.")
                continue

            # Heading (NAV-RELPOSNED)
            if msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                heading, heading_acc = _auto_heading_deg(
                    getattr(msg, "relPosHeading", None),
                    getattr(msg, "accHeading", None),
                )
                continue

            # Position/time (NAV-PVT)
            if msg.identity == "NAV-PVT":
                t = None
                if getattr(msg, "validTime", 0):
                    t = datetime(
                        msg.year, msg.month, msg.day,
                        msg.hour, msg.min, msg.second,
                        tzinfo=timezone.utc
                    ) + timedelta(microseconds=int(getattr(msg, "nano", 0) / 1000))

                fixType = getattr(msg, "fixType", 0)
                numSV = getattr(msg, "numSV", None)

                lat = _auto_deg(getattr(msg, "lat", None))
                lon = _auto_deg(getattr(msg, "lon", None))
                alt = _auto_alt_m(getattr(msg, "height", None))

                line = f"[{t.isoformat() if t else 'no-time'}] fix={FIXTYPE_MAP.get(fixType, fixType)}"
                if numSV is not None:
                    line += f", sats_used={numSV}"

                if fixType >= 2 and lat is not None and lon is not None:
                    line += f", lat={lat:.7f}, lon={lon:.7f}"
                    if alt is not None:
                        line += f", alt={alt:.2f} m"

                if heading is not None:
                    line += f", heading={heading:.2f}° ±{(heading_acc or 0.0):.2f}°"

                print(line)

def print_ports():
    print("Available ports:")
    for p in list_ports.comports():
        print(f" - {p.device} | {p.description}")

if __name__ == "__main__":
    print_ports()

    PORT = "COM6"
    BAUD = 38400  # try 115200 if needed

    try:
        stream_status(port=PORT, baud=BAUD, timeout=1.0)
    except KeyboardInterrupt:
        print("\n[GPS] Stopped.")
