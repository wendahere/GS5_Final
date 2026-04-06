# gps_satellite_full.py
# u-blox GNSS: position + heading + satellites USED in solution
# Requires: pip install pyserial pyubx2

from serial import Serial
from serial.tools import list_ports
from datetime import datetime, timezone, timedelta
from pyubx2 import UBXReader, UBXMessage

# ---------------- GNSS Maps ----------------

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

# ---------------- Helper Functions ----------------

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
        heading = (heading - 90.0) % 360.0  # adjust if needed for your baseline orientation

    return heading, acc


def _sat_used_flag(s):
    """
    Determine if satellite is used in navigation solution.
    Compatible with different firmware/pyubx2 versions.
    """
    # Direct boolean fields (varies by version)
    for attr in ("svUsed", "used", "usedInNav", "usedInSolution"):
        if hasattr(s, attr):
            v = getattr(s, attr)
            if isinstance(v, bool):
                return v

    # Most common: flags bitfield (bit0 = used)
    flags = getattr(s, "flags", None)
    if isinstance(flags, int):
        return bool(flags & 0x01)

    return None


def _fmt_sat(s):
    gnss = GNSS_MAP.get(getattr(s, "gnssId", None), str(getattr(s, "gnssId", "?")))
    sv   = getattr(s, "svId", "?")
    cno  = getattr(s, "cno", None)
    elev = getattr(s, "elev", None)
    azim = getattr(s, "azim", None)

    used = _sat_used_flag(s)
    if used is True:
        used_txt = "USED"
    elif used is False:
        used_txt = "----"
    else:
        used_txt = "????"

    return f"{gnss:7} SV{sv:>3}  {used_txt}  C/N0={str(cno):>2} dBHz  elev={str(elev):>3}°  az={str(azim):>3}°"


# ---------------- Main Streaming Function ----------------

def stream_status(port="COM6", baud=38400, timeout=1.0, sat_print_every_s=2.0, sat_top_n=12):

    with Serial(port, baud, timeout=timeout) as s:

        # Accept UBX + NMEA + RTCM (we ignore RTCM)
        ubr = UBXReader(s, protfilter=7)

        heading = heading_acc = None
        last_sat_print = datetime.now(timezone.utc)

        print(f"\n[GPS] Listening on {port}@{baud} (Ctrl+C to stop)")
        print("-" * 75)

        while True:

            try:
                raw, msg = ubr.read()
            except Exception:
                continue  # ignore decode errors (older pyubx2)

            # Ignore RTCM frames (0xD3)
            if raw and raw[:1] == b"\xD3":
                continue

            if not isinstance(msg, UBXMessage):
                continue

            # ---------------- SATELLITES ----------------
            if msg.identity == "NAV-SAT":
                now = datetime.now(timezone.utc)

                if now - last_sat_print >= timedelta(seconds=sat_print_every_s):
                    last_sat_print = now

                    sats = getattr(msg, "sats", []) or []
                    if not sats:
                        continue

                    sats_sorted = sorted(
                        sats,
                        key=lambda x: getattr(x, "cno", 0) or 0,
                        reverse=True,
                    )

                    used_sats = []
                    other_sats = []

                    for srec in sats_sorted:
                        if _sat_used_flag(srec):
                            used_sats.append(srec)
                        else:
                            other_sats.append(srec)

                    print("\n[SAT] Satellites USED in solution:")
                    if used_sats:
                        for srec in used_sats[:sat_top_n]:
                            print(" ", _fmt_sat(srec))
                    else:
                        print("  (No satellites flagged as USED)")

                    print("\n[SAT] Other satellites in view:")
                    for srec in other_sats[:sat_top_n]:
                        print(" ", _fmt_sat(srec))

                continue

            # ---------------- HEADING ----------------
            if msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                heading, heading_acc = _auto_heading_deg(
                    getattr(msg, "relPosHeading", None),
                    getattr(msg, "accHeading", None),
                )
                continue

            # ---------------- POSITION ----------------
            if msg.identity == "NAV-PVT":

                t = None
                if getattr(msg, "validTime", 0):
                    t = datetime(
                        msg.year, msg.month, msg.day,
                        msg.hour, msg.min, msg.second,
                        tzinfo=timezone.utc
                    ) + timedelta(
                        microseconds=int(getattr(msg, "nano", 0) / 1000)
                    )

                fixType = getattr(msg, "fixType", 0)
                numSV = getattr(msg, "numSV", None)

                lat = _auto_deg(getattr(msg, "lat", None))
                lon = _auto_deg(getattr(msg, "lon", None))
                alt = _auto_alt_m(getattr(msg, "height", None))

                line = f"[{t.isoformat() if t else 'no-time'}] "
                line += f"fix={FIXTYPE_MAP.get(fixType, fixType)}"

                if numSV is not None:
                    line += f", sats_used={numSV}"

                if fixType >= 2 and lat is not None and lon is not None:
                    line += f", lat={lat:.7f}, lon={lon:.7f}"
                    if alt is not None:
                        line += f", alt={alt:.2f} m"

                if heading is not None:
                    line += f", heading={heading:.2f}° ±{(heading_acc or 0.0):.2f}°"

                print(line)


# ---------------- Entry Point ----------------

def print_ports():
    print("Available ports:")
    for p in list_ports.comports():
        print(f" - {p.device} | {p.description}")


if __name__ == "__main__":

    print_ports()

    PORT = "COM6"       # change if needed
    BAUD = 38400        # try 115200 if needed

    try:
        stream_status(port=PORT, baud=BAUD)
    except KeyboardInterrupt:
        print("\n[GPS] Stopped.")
