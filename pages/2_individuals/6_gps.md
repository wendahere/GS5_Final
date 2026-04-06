---
title: Global Positioning System 
parent: Work Area
nav_order: 6
permalink: /GPS/
---



# **6\. Global Positioning System** 

## **6.1 Objective**

The Global Positioning System (GPS) module within the ground station is essential to provide accurate real-time position and heading information before we can begin satellite tracking operations. We need to know the exact location of the ground station and where it is facing as we must establish a ground position relative to the satellite that we are tracking. This step is required as the ground station is portable and the location of it will vary.

The objective of this global positioning system is to implement and calibrate a GPS-based positioning subsystem that interfaces and works with the gimbal control software. This integration ensures that the ground station can maintain consistent tracking accuracy, making the entire system work even when deployed in different locations.

## **6.2 Design Specifications** 

The Global Positioning System (GPS) module shall fulfil the following design requirements to ensure accurate and reliable position and heading determination for satellite tracking operations:

* **GPS Module:** AS-STARTKIT-HEAD-L1L2-NH-00

* **Heading Accuracy:** \< 0.2°

* **Position Accuracy:** Within ±2.5 m

* **Update Rate:** Minimum 10 Hz to support real-time gimbal motion updates

* **Communication Interface:** Serial (UART) connection to the main control unit

* **Power Supply:** 5 V DC regulated input

* **Integration Requirement:** Must interface seamlessly with the TLE parsing and gimbal control subsystems for continuous azimuth and elevation computation.

The selected GPS module meets the above specifications, providing dual-frequency L1/L2 capability for improved heading precision and resistance to multipath errors. These specifications collectively ensure stable and accurate satellite tracking performance in mobile ground station configurations.

## **6.3 Code Required**

The GPS data acquisition and validation were implemented in Python.  
The serial port verification script (`check_ports.py`) and the GPS parsing and heading computation module (`gps_reader.py`) are provided in Gimbal Control Software Folder for reference.
[Gimbal Control Software Folder](https://github.com/wendahere/GS5/tree/main/pages/5_appendices/Gimbal%20Code%20Listing) - <h>(This will bring you to a github folder containing all the code)</h>

A. `check_ports.py` \- Serial Port Discovery   
Purpose:

Provides a quick way to list all available serial ports on the host machine and verify that the target GPS device/port can be opened at the expected settings (baud rate, timeout). 

This is used before running the live GPS reader to avoid silent connection failures.

Key Components

* Port enumeration: Uses `serial.tools.list_ports.comports()` to print detected ports and human-readable descriptions.

* Connection probe: Attempts to open a candidate list of device names for Windows (`"COM6"` and the escaped `"\\.\COM6"`) at 38400 baud with a 2 second timeout.

* Result reporting: Prints “Opened OK” / “Open failed” with the exception text to aid troubleshooting.

Inputs / Parameters

* `target = "COM6"` \- change to match the actual GPS port.

* `baud = 38400` (fixed in code) \- must match the GPS module’s configured baud rate.

Outputs

* Console log listing ports and the success/failure of opening each candidate device.

B. `gps_reader.py` \- UBX-First GPS Fix & Heading Utility  
Purpose

Acquire a single high-quality GPS fix (time, latitude, longitude, altitude) using a dual-antenna setup to find the heading. Prioritises u-blox UBX messages for robustness and falls back to National Marine Electronics Association (NMEA )if needed.

External Dependencies

* `pyserial` (serial I/O)

* `pyubx2` (parsing u-blox UBX protocol)

Module Structure

1\) Helper Converters

* `_auto_deg(v)` and `_auto_alt_m(h)`

  * Auto-scale raw UBX integer fields (e.g., `1e-7 deg` for lat/lon, millimetres to metres for height).

* `_auto_heading_deg(rh, ah)`

  * Scales UBX relative heading and heading accuracy (from `1e-5 deg`), then normalises heading with a `–90°` frame adjustment to align with the project’s heading convention.

* `_dm_to_deg(dm, hemi)`

  * Converts NMEA “degrees+minutes” strings into signed decimal degrees using hemisphere flags.

2\) Public API

* `read_once(port="COM6", baud=38400, max_wait_s=8.0, want_heading=True, extra_heading_wait_s=2.0)`

  * Goal: Return one consolidated fix as a Python dict:  
     `{time_utc, lat, lon, alt_m, heading_deg, heading_acc_deg, source}`.

  * Acquisition strategy:

    1. Prefer UBX `NAV-PVT` for position & time (requires `fixType ≥ 2`).

    2. Listen opportunistically for UBX `NAV-RELPOSNED` to obtain dual-antenna heading and its accuracy.

    3. Fallback to NMEA (`GGA` or `GLL`) for position if UBX is not yet available within the wait window.

    4. If position is obtained but heading is missing, keep listening up to `extra_heading_wait_s` for a late `RELPOSNED`.

  * Timeout behaviour: Raises a `TimeoutError` if no position is acquired before `max_wait_s`.

  * Source flag: Sets `source = "UBX"`, `"NMEA-GGA"`, or `"NMEA-GLL"` to document where the fix came from.

* `stream_status(port="COM6", baud=38400, timeout=1.0)`

  * Live console monitor for continuous UBX telemetry.

  * Prints fix type (e.g., “2D/3D”), satellite count, position, altitude, and (when available) heading with ±accuracy.

  * Useful for antenna placement tests, sky-view diagnostics, and verifying heading accuracy.

3\) Command-Line Entry Point

When executed directly:

1. Lists available ports; 2\) Calls `read_once(...)`; 3\) Prints a formatted “GPS FIX” summary; 4\) (Optionally) enables `stream_status(...)` for continuous monitoring.

Data & Accuracy Notes

* Heading source: Relies on UBX `NAV-RELPOSNED` (dual-antenna/RTK-capable receivers) to obtain true heading and a manufacturer-reported heading accuracy (`heading_acc_deg`).

* Time handling: Prefers UBX-validated time; on NMEA fallback, constructs a UTC timestamp from sentence fields for logging consistency.

* Units & normalisation: All lat/lon in decimal degrees, altitude in metres, heading in degrees 0–360.

Typical Use in the System

1. Bring-up: Use `check_ports.py` to confirm COM assignment.

2. Single-fix acquisition: Call `read_once(...)` from the tracking stack to seed ground-station location/time (and heading if available).

3. Live QA: Run `stream_status(...)` to evaluate antenna placement and confirm that heading ± accuracy meets operational thresholds before a satellite pass.
