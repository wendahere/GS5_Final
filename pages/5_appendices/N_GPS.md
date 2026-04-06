---
title: "Appendix N: GNSS Satellite Monitoring Code (ZED-F9P)"
parent: Appendices
nav_order: 14
permalink: /GPS2/

---
**Appendix N: GNSS Satellite Monitoring Code (ZED-F9P)**  
\# Displays position, heading, and satellites used in solution

from serial import Serial  
from serial.tools import list\_ports  
from datetime import datetime, timezone  
from pyubx2 import UBXReader, UBXMessage

def auto\_deg(v):  
    return (v \* 1e-7) if v else None

def auto\_alt(v):  
    return (v / 1000.0) if v else None

def auto\_heading(rh, ah):  
    if rh is None:  
        return None, None  
    heading \= (rh \* 1e-5 \- 90.0) % 360  
    acc \= ah \* 1e-5 if ah else None  
    return heading, acc

def parse\_nmea(raw):  
    if raw and raw\[:1\] \== b"$":  
        try:  
            return raw.decode("ascii", errors="ignore").strip()  
        except:  
            return None  
    return None

def parse\_gsa(line):  
    parts \= line.split(",")  
    return {p.split("\*")\[0\] for p in parts\[3:15\] if p}

def parse\_gsv(line):  
    parts \= line.split(",")  
    sats \= \[\]  
    i \= 4  
    while i \+ 3 \< len(parts):  
        prn \= parts\[i\]  
        elev \= parts\[i+1\]  
        azim \= parts\[i+2\]  
        snr \= parts\[i+3\].split("\*")\[0\]  
        if prn:  
            sats.append((prn, elev, azim, snr))  
        i \+= 4  
    return sats

def stream\_status(port="COM6", baud=38400):

    with Serial(port, baud, timeout=1.0) as s:  
        ubr \= UBXReader(s, protfilter=7)

        used \= set()  
        sats \= {}  
        heading \= None  
        acc \= None

        while True:

            raw, msg \= ubr.read()

            \# Ignore RTCM  
            if raw and raw\[:1\] \== b"\\xD3":  
                continue

            \# \---------- NMEA \----------  
            line \= parse\_nmea(raw)

            if line:  
                if "GSA" in line:  
                    used \= parse\_gsa(line)

                elif "GSV" in line:  
                    for prn, elev, azim, snr in parse\_gsv(line):  
                        sats\[prn\] \= (elev, azim, snr)

                print("\\n\[SATELLITES\]")  
                for prn in sorted(sats):  
                    elev, azim, snr \= sats\[prn\]  
                    flag \= "USED" if prn in used else "----"  
                    print(f"PRN {prn:\>3}  {flag}  SNR={snr}  elev={elev}  az={azim}")

            \# \---------- UBX \----------  
            if isinstance(msg, UBXMessage):

                if msg.identity \== "NAV-RELPOSNED" and msg.relPosValid:  
                    heading, acc \= auto\_heading(msg.relPosHeading, msg.accHeading)

                if msg.identity \== "NAV-PVT":

                    t \= datetime(  
                        msg.year, msg.month, msg.day,  
                        msg.hour, msg.min, msg.second,  
                        tzinfo=timezone.utc  
                    )

                    lat \= auto\_deg(msg.lat)  
                    lon \= auto\_deg(msg.lon)  
                    alt \= auto\_alt(msg.height)

                    print("\\n--- GPS FIX \---")  
                    print("UTC Time :", t)  
                    print("Latitude :", lat)  
                    print("Longitude:", lon)  
                    print("Altitude :", alt)

                    if heading is not None:  
                        print(f"Heading  : {heading:.2f}° ±{acc:.2f}°")

if \_\_name\_\_ \== "\_\_main\_\_":

    print("Available ports:")  
    for p in list\_ports.comports():  
        print(f" \- {p.device}")

    stream\_status("COM6", 38400\)  