---
title: "Appendix J - TLE_Set_Builder.py"
parent: Appendices
nav_order: 10
permalink: /appendix-bf/

---

## Appendix J - TLE_Set_Builder.py - Daily TLE Fetch

```
#!/usr/bin/env python3
"""
TLE_Set_Builder.py
- Fetches latest TLE data from CelesTrak
- Saves to TLE/TLEYYYY-MM-DD.tle
"""

import requests
import datetime
from pathlib import Path

# ---------------- USER SETTINGS ----------------
# Pick a CelesTrak source (change if you need a different group)
TLE_URL = "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle"

OUTPUT_DIR = Path(__file__).parent / "TLE"
# ------------------------------------------------

def main():
    today = datetime.date.today()
    tle_path = OUTPUT_DIR / f"TLE{today}.tle"

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Downloading TLEs from {TLE_URL} …")
    r = requests.get(TLE_URL, timeout=10)
    r.raise_for_status()
    tle_text = r.text.strip()

    # Save to file
    with open(tle_path, "w") as f:
        f.write(tle_text)

    print(f"[OK] Saved {len(tle_text.splitlines())//3} satellites to {tle_path}")

if __name__ == "__main__":
    main()

```