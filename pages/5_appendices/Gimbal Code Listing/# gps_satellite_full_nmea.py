# gps_satellite_full_nmea.py
# u-blox GNSS: UBX position + heading, and NMEA satellite list (GSV) + USED sats (GSA)
# deps: pip install pyserial pyubx2

from serial import Serial
from serial.tools import list_ports
from datetime import datetime, timezone, timedelta
from pyubx2 import UBXReader, UBXMessage

FIXTYPE_MAP = {
    0: "No fix",
    1: "DR only",
    2: "2D",
    3: "3D",
    4: "GNSS+DR",
    5: "Time only",
}

# ---------------- helpers (UBX) ----------------

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
        heading = (heading - 90.0) % 360.0  # adjust only if you intentionally want this rotation
    return heading, acc

# ---------------- helpers (NMEA) ----------------

def _parse_nmea_line(raw: bytes) -> str | None:
    if not raw:
        return None
    if raw[:1] != b"$":
        return None
    try:
        return raw.decode("ascii", errors="ignore").strip()
    except Exception:
        return None

def _talker(line: str) -> str:
    # "$GPGSV" -> "GP", "$GNGSA" -> "GN"
    if len(line) >= 3 and line[0] == "$":
        return line[1:3]
    return "??"

def _parse_gsa_used_prns(line: str):
    """
    Parse GSA to extract PRNs used in fix.
    Example: $GNGSA,A,3,76,65,88,66,78,,,,,,,,1.76,1.15,1.33*..
    PRNs are fields 3..14 (12 fields)
    """
    parts = line.split(",")
    if len(parts) < 15:
        return []
    talk = _talker(line)  # GP/GL/GA/GB/GN etc
    prns = []
    # fields 3..14 inclusive
    for i in range(3, 15):
        if i < len(parts):
            p = parts[i].split("*")[0].strip()
            if p:
                prns.append((talk, p))
    return prns

def _parse_gsv_sats(line: str):
    """
    Parse GSV: satellites in view.
    Example: $GPGSV,3,1,10,10,73,128,47,16,33,206,47,...
    Each sat block: PRN, elev, azim, SNR
    """
    parts = line.split(",")
    if len(parts) < 4:
        return []
    talk = _talker(line)
    sats = []
    # Sat blocks start at field 4, each 4 fields
    i = 4
    while i + 3 < len(parts):
        prn = parts[i].strip()
        elev = parts[i+1].strip()
        azim = parts[i+2].strip()
        snr = parts[i+3].split("*")[0].strip()  # remove checksum
        if prn:
            sats.append({
                "talker": talk,
                "prn": prn,
                "elev": int(elev) if elev.isdigit() else None,
                "azim": int(azim) if azim.isdigit() else None,
                "snr": int(snr) if snr.isdigit() else None,
            })
        i += 4
    return sats

def _constellation_name_from_talker(talk: str) -> str:
    # Note: GN = "mixed", not a real constellation; it means combined NMEA output.
    return {
        "GP": "GPS",
        "GL": "GLONASS",
        "GA": "Galileo",
        "GB": "BeiDou",
        "BD": "BeiDou",
        "GQ": "QZSS",
        "GN": "GNSS",
    }.get(talk, talk)

# ---------------- main ----------------

def stream_status(port="COM6", baud=38400, timeout=1.0, sat_print_every_s=2.0, sat_top_n=16):
    with Serial(port, baud, timeout=timeout) as s:
        ubr = UBXReader(s, protfilter=7)  # UBX + NMEA + RTCM

        heading = heading_acc = None

        # NMEA satellite caches
        used_prns = set()          # set of (talker, prn) from GSA
        sats_in_view = {}          # key (talker, prn) -> sat dict from GSV

        last_sat_print = datetime.now(timezone.utc)

        print(f"\n[GPS] Listening on {port}@{baud} (Ctrl+C to stop)")
        print("-" * 80)

        while True:
            try:
                raw, msg = ubr.read()
            except Exception:
                continue

            # Ignore RTCM frames (RTCM3 starts with 0xD3)
            if raw and raw[:1] == b"\xD3":
                continue

            # ---- NMEA parse for satellites ----
            line = _parse_nmea_line(raw)
            if line:
                if line.startswith(("$GNGSA", "$GPGSA", "$GLGSA", "$GAGSA", "$GBGSA", "$BDGSA", "$GQGSA")):
                    for tp in _parse_gsa_used_prns(line):
                        used_prns.add(tp)

                elif line.startswith(("$GNGSV", "$GPGSV", "$GLGSV", "$GAGSV", "$GBGSV", "$BDGSV", "$GQGSV")):
                    for sat in _parse_gsv_sats(line):
                        key = (sat["talker"], sat["prn"])
                        sats_in_view[key] = sat

                # print satellites periodically (even if UBX doesn't send NAV-SAT)
                now = datetime.now(timezone.utc)
                if now - last_sat_print >= timedelta(seconds=sat_print_every_s):
                    last_sat_print = now

                    # Build printable list sorted by SNR
                    rows = []
                    for (talk, prn), sat in sats_in_view.items():
                        used = (talk, prn) in used_prns or ("GN", prn) in used_prns
                        rows.append((used, sat.get("snr") or -1, talk, prn, sat.get("elev"), sat.get("azim")))

                    rows.sort(key=lambda x: (x[0], x[1]), reverse=True)

                    print("\n[SAT] Satellites (from NMEA GSV/GSA). USED = contributing to current fix:")
                    if not rows:
                        print("  (No GSV seen yet. If you only output UBX, enable NMEA GSV/GSA or UBX NAV-SAT.)")
                    else:
                        for used, snr, talk, prn, elev, azim in rows[:sat_top_n]:
                            const = _constellation_name_from_talker(talk)
                            used_txt = "USED" if used else "----"
                            snr_txt = f"{snr:>2}dBHz" if snr >= 0 else "  ?dBHz"
                            elev_txt = f"{elev:>3}°" if elev is not None else "  ?°"
                            azim_txt = f"{azim:>3}°" if azim is not None else "  ?°"
                            print(f"  {const:8} PRN{prn:>3}  {used_txt}  SNR={snr_txt}  elev={elev_txt}  az={azim_txt}")

            # ---- UBX handling for heading/position ----
            if isinstance(msg, UBXMessage):
                # Heading
                if msg.identity == "NAV-RELPOSNED" and getattr(msg, "relPosValid", 0):
                    heading, heading_acc = _auto_heading_deg(
                        getattr(msg, "relPosHeading", None),
                        getattr(msg, "accHeading", None),
                    )
                    continue

                # Position
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

                    line_out = f"[{t.isoformat() if t else 'no-time'}] fix={FIXTYPE_MAP.get(fixType, fixType)}"
                    if numSV is not None:
                        line_out += f", sats_used={numSV}"
                    if fixType >= 2 and lat is not None and lon is not None:
                        line_out += f", lat={lat:.7f}, lon={lon:.7f}"
                        if alt is not None:
                            line_out += f", alt={alt:.2f} m"
                    if heading is not None:
                        line_out += f", heading={heading:.2f}° ±{(heading_acc or 0.0):.2f}°"
                    print(line_out)

def print_ports():
    print("Available ports:")
    for p in list_ports.comports():
        print(f" - {p.device} | {p.description}")

if __name__ == "__main__":
    print_ports()

    PORT = "COM6"
    BAUD = 38400  # if nothing comes, try 115200

    try:
        stream_status(port=PORT, baud=BAUD, timeout=1.0)
    except KeyboardInterrupt:
        print("\n[GPS] Stopped.")
