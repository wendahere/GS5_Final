# -*- coding: utf-8 -*-
"""
Simplified 2-Axis Gimbal Controller (AZ/EL) — NSC-G3-E / Galil-safe

Enhancements:
- Adds streaming-safe Position Tracking (PT) mode for continuous target updates
  without BG/AM/MC, preventing TC:7 ("not valid while running") during PCHIP streams.
- Keeps classic PA/PR + BG flows for non-streaming helpers (move_absolute, move_relative).
- Uses _TNX/_TNY polling for waits (as before) and adds a settle loop for PT.

Key choices (unchanged unless noted):
- Classic PA/PR/BG supported.
- AC/DC/SP classic 'AC x,y', 'DC x,y', 'SP x,y'.
- One command per line (no semicolons).
- Waits via _TNX/_TNY polling (no AMX/AMY).
- Light connection retry (given string → raw IP → '-d').
"""

import time
import numpy as np
from pathlib import Path

try:
    import gclib
except ImportError:
    gclib = None  # simulation mode if library isn't present

# ------------------- USER LIMITS -------------------
AZ_MIN, AZ_MAX = -90.0, 90.0
EL_MIN, EL_MAX = -90.0, 90.0
MAX_STEP_DEG = None  # reserved for future segmentation
# ---------------------------------------------------

STATE_FILE = "dependencies/gimbal_state/currGimbalPosition.txt"
MIN_PR_COUNTS = 10    # deadband to avoid tiny moves
MIN_PA_DELTA = 6      # counts; ignore microscopic PA updates in PT

AXIS_AZ = 'X'
AXIS_EL = 'Y'

# Counts-per-degree (calibrated)
CNT_PER_DEG_AZ = 10000
CNT_PER_DEG_EL = 10000


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def _safe_load_pos():
    try:
        p = Path(STATE_FILE)
        if not p.exists():
            return [0.0, 0.0, 0.0]
        vals = np.loadtxt(p)
        return [float(vals[0]), float(vals[1]), float(vals[2])]
    except Exception:
        return [0.0, 0.0, 0.0]


def _safe_save_pos(vec3):
    try:
        Path(STATE_FILE).parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(STATE_FILE, np.asarray(vec3))
    except Exception as e:
        print(f"[WARN] Could not save position: {e}")


class GimbalController:
    def __init__(self, connection, cnt_per_deg=(CNT_PER_DEG_AZ, CNT_PER_DEG_EL),
                 assume_zero_on_connect=True, streaming=True):
        """
        connection: e.g. '192.168.1.2' or '192.168.1.2 -d'
        assume_zero_on_connect=True:
            Define current counts as 0,0 on connect without moving hardware (DP 0,0).
        streaming=True:
            Enable Position Tracking (PT) mode so PA targets can be updated DURING motion
            with no BG — ideal for high-rate PCHIP streams.
        """
        print("[INIT] Connecting to Galil Controller...")
        self.cnt_az, self.cnt_el = cnt_per_deg
        self.curr_az, self.curr_el = 0.0, 0.0
        self.curr_pos = _safe_load_pos()
        self._assume_zero = assume_zero_on_connect
        self.streaming = streaming

        self.sim = gclib is None
        if self.sim:
            print("[SIMULATION MODE] No gclib available; motions won't go to hardware.")
            return

        # --- Light connection retry for robustness ---
        self.g = gclib.py()
        variants = []
        s = str(connection).strip()
        parts = s.split()

        if s:
            variants.append(s)  # as provided
        if parts:
            ip_only = parts[0]
            if ip_only not in variants:
                variants.append(ip_only)
            dash_d = f"{ip_only} -d"
            if dash_d not in variants:
                variants.append(dash_d)

        last_err = None
        for cand in variants:
            try:
                self.g.GOpen(cand)
                try:
                    self.g.timeout = 120000  # ms
                except Exception:
                    pass
                try:
                    print(self.g.GInfo().strip())
                except Exception:
                    print("[INFO] Connected.")
                break
            except Exception as e:
                last_err = e
                try:
                    self.g.GClose()
                except Exception:
                    pass
                time.sleep(0.2)
        else:
            raise RuntimeError(f"Failed to connect using {variants}: {last_err}")

        # Bring controller to a known state and set motion params
        self._setup_motion()

        # Enter PT if streaming (so PA can be updated during motion)
        if self.streaming:
            self._enter_pt()

        # Sync our object state from controller (counts → deg)
        self._sync_from_controller()

    # -------------- low-level I/O --------------
    def _cmd(self, s, quiet=False):
        """Send a raw Galil command, with '?' handling that also fetches TC when possible."""
        try:
            resp = self.g.GCommand(s)
            if not quiet:
                msg = resp.strip().replace("\r", " ").replace("\n", " ")
                print(f"[GALIL] {s}" + ("" if not msg else f" -> {msg}"))
            return resp
        except gclib.GclibError as e:
            # Try to fetch TC code for diagnostics
            try:
                tc = self.g.GCommand("TC").strip()
            except Exception:
                tc = "<TC read failed>"
            print(f"[GALIL ERROR] {s}\n  → {e}\n  → TC: {tc}")
            raise
        except Exception:
            print(f"[GALIL ERROR] {s}")
            raise

    # -------------- PT helpers --------------
    def _enter_pt(self):
        """Enable Position Tracking mode (validates PA during motion, no BG)."""
        try:
            # Ensure idle profiler before enabling PT
            self._cmd("ST", quiet=True)
            self._cmd("PT 1,1", quiet=True)
            print("[OK] PT mode enabled for streaming PA updates.")
        except Exception:
            print("[WARN] Could not enable PT mode; continuing classic mode.")
            self.streaming = False

    def _exit_pt(self):
        """Disable Position Tracking mode to allow classic PA/PR + BG motion."""
        try:
            self._cmd("ST", quiet=True)
            self._cmd("PT 0,0", quiet=True)
            print("[OK] PT mode disabled (classic BG motion allowed).")
        except Exception:
            pass

    # -------------- setup / sync --------------
    def _set_params(self):
        """Set motion parameters using classic X,Y forms (no named-axis)."""
        self._cmd("AB", quiet=True)          # abort any left-over program/motion
        self._cmd("ST", quiet=True)          # stop
        self._cmd("WT 20", quiet=True)       # small wait

        # Motor off (combined, then per-axis fallbacks)
        try:
            self._cmd("MO XY", quiet=True)
        except Exception:
            try: self._cmd("MO X", quiet=True)
            except Exception: pass
            try: self._cmd("MO Y", quiet=True)
            except Exception: pass

        # Define current commanded position = 0,0
        if self._assume_zero:
            try:
                self._cmd("DP 0,0", quiet=True)
            except Exception:
                # Per-axis as fallback
                try: self._cmd("DP X=0", quiet=True)
                except Exception: pass
                try: self._cmd("DP Y=0", quiet=True)
                except Exception: pass

        # Servo on (combined, then per-axis fallbacks)
        try:
            self._cmd("SH XY", quiet=True)
        except Exception:
            try: self._cmd("SH X", quiet=True)
            except Exception: pass
            try: self._cmd("SH Y", quiet=True)
            except Exception: pass

        # Motion parameters — classic comma syntax (firmware-safe)
        self._cmd("AC 200000,200000", quiet=True)  # counts/s^2
        self._cmd("DC 200000,200000", quiet=True)  # counts/s^2
        self._cmd("SP 60000,60000",   quiet=True)  # counts/s

        print("[OK] Motion parameters set." + (" (DP 0,0 applied)" if self._assume_zero else ""))

    def _setup_motion(self):
        """Init: stop, zero (optional), servo on, params (classic syntax)."""
        self._set_params()

    def _sync_from_controller(self):
        """Read TP counts and convert to our deg state."""
        cx, cy = 0.0, 0.0
        try:
            cx = float(self._cmd("TP X", quiet=True).strip())
        except Exception:
            pass
        try:
            cy = float(self._cmd("TP Y", quiet=True).strip())
        except Exception:
            pass
        self.curr_az = cx / self.cnt_az
        self.curr_el = cy / self.cnt_el
        self.curr_pos = [self.curr_az, self.curr_el, 0.0]
        print(f"[SYNC] AZ={self.curr_az:.3f}°, EL_gimbal={self.curr_el:.3f}°")

    def close(self):
        if not self.sim:
            try:
                self._cmd("ST", quiet=True)
            except Exception:
                pass
            # Back out of PT to leave controller in classic state
            if self.streaming:
                self._exit_pt()
            try:
                self.g.GClose()
            except Exception:
                pass
        _safe_save_pos(self.curr_pos)
        print("[CLOSED] Gimbal safely disconnected.")

    # -------------- waits / health --------------
    def _wait_inpos(self, timeout_s=2.0):
        """
        Non-blocking in-position wait using _TNX/_TNY (>=1 means in position).
        Avoids AMX/AMY which have caused timeouts on some units.
        """
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                tn = self._cmd("MG _TNX,_TNY", quiet=True).strip().split()
                if len(tn) >= 2 and all(float(x) >= 1 for x in tn[:2]):
                    return True
            except Exception:
                pass
            time.sleep(0.02)
        return False

    def _wait_settle_counts(self, tgt_x, tgt_y, timeout_s=2.0, tol_counts=30):
        """
        PT settle helper: poll TP until within tolerance of target counts.
        """
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                cx = float(self._cmd("TP X", quiet=True).strip())
                cy = float(self._cmd("TP Y", quiet=True).strip())
                if abs(cx - tgt_x) <= tol_counts and abs(cy - tgt_y) <= tol_counts:
                    return True
            except Exception:
                pass
            time.sleep(0.02)
        return False

    # -------------- motion helpers --------------
    def _send_relative_counts(self, dcnt_x, dcnt_y, wait=True):
        """Send PR using classic comma syntax; then BG XY."""
        mv_x = abs(dcnt_x) >= MIN_PR_COUNTS
        mv_y = abs(dcnt_y) >= MIN_PR_COUNTS
        if not (mv_x or mv_y):
            return False

        # Classic comma form: zeros for non-moving axes
        pr_x = int(dcnt_x) if mv_x else 0
        pr_y = int(dcnt_y) if mv_y else 0

        # If currently in PT, temporarily exit to allow BG motion
        was_pt = self.streaming
        if was_pt:
            self._exit_pt()

        self._cmd(f"PR {pr_x},{pr_y}", quiet=True)
        self._cmd("BG XY", quiet=True)

        if wait:
            self._wait_inpos(timeout_s=2.0)

        # Re-enter PT if we were streaming before
        if was_pt:
            self._enter_pt()

        return True

    def _send_absolute_counts(self, tgt_x, tgt_y, wait=True):
        """
        Absolute move.
        - In streaming/PT mode: issue PA only (no BG); optionally wait via settle loop.
        - In classic mode: PA + BG, then _TN waits.
        """
        if self.streaming:
            # Drop tiny updates to avoid saturating the bus with negligible changes
            try:
                cx = float(self._cmd("TP X", quiet=True).strip())
                cy = float(self._cmd("TP Y", quiet=True).strip())
            except Exception:
                cx, cy = None, None

            if cx is not None and cy is not None:
                if abs(tgt_x - cx) < MIN_PA_DELTA and abs(tgt_y - cy) < MIN_PA_DELTA:
                    return False

            self._cmd(f"PA {int(tgt_x)},{int(tgt_y)}", quiet=True)
            if wait:
                self._wait_settle_counts(tgt_x, tgt_y, timeout_s=2.0)
            return True

        # Classic (non-PT)
        self._cmd(f"PA {int(tgt_x)},{int(tgt_y)}", quiet=True)
        self._cmd("BG XY", quiet=True)
        if wait:
            self._wait_inpos(timeout_s=2.0)
        return True

    # -------------- public movement API --------------
    def move_absolute(self, az_deg, el_sky_deg, eps_deg=1e-6, wait=True):
        """
        Move to absolute AZ / EL_sky.
        NOTE: If you feed sky elevation here, internal convention assumes:
              EL_gimbal = EL_sky - 90°
        """
        target_az = _clip(float(az_deg), AZ_MIN, AZ_MAX)
        target_el = _clip(float(el_sky_deg) - 90.0, EL_MIN, EL_MAX)

        if self.sim:
            self.curr_az, self.curr_el = target_az, target_el
            self.curr_pos = [self.curr_az, self.curr_el, 0.0]
            return

        curr_cnt_x = int(round(self.curr_az * self.cnt_az))
        curr_cnt_y = int(round(self.curr_el * self.cnt_el))
        tgt_cnt_x  = int(round(target_az * self.cnt_az))
        tgt_cnt_y  = int(round(target_el * self.cnt_el))

        if (tgt_cnt_x == curr_cnt_x) and (tgt_cnt_y == curr_cnt_y):
            return

        # Use classic PA+BG for one-shot absolute moves (exit PT temporarily if needed)
        was_pt = self.streaming
        if was_pt:
            self._exit_pt()
        self._send_absolute_counts(tgt_cnt_x, tgt_cnt_y, wait=wait)  # classic branch inside
        if was_pt:
            self._enter_pt()

        self.curr_az = target_az
        self.curr_el = target_el
        self.curr_pos = [self.curr_az, self.curr_el, 0.0]

    def move_relative(self, d_az, d_el_sky, wait=True):
        """
        Relative movement (ΔAz_deg, ΔEl_sky_deg). Limits enforced.
        """
        new_az = _clip(self.curr_az + float(d_az), AZ_MIN, AZ_MAX)
        new_el = _clip(self.curr_el + float(d_el_sky), EL_MIN, EL_MAX)

        if self.sim:
            self.curr_az, self.curr_el = new_az, new_el
            self.curr_pos = [new_az, new_el, 0.0]
            return

        dcnt_x = int(round((new_az - self.curr_az) * self.cnt_az))
        dcnt_y = int(round((new_el - self.curr_el) * self.cnt_el))

        if self._send_relative_counts(dcnt_x, dcnt_y, wait=wait):
            self.curr_az, self.curr_el = new_az, new_el
            self.curr_pos = [new_az, new_el, 0.0]

    def degSteer(self, az_gim, el_gim, absolute=True, wait=False, eps_deg=1e-6):
        """
        Steer using *gimbal-frame* degrees (AZ: -90..+90, EL: -90..+90).
        absolute=True  → target absolute (recommended for streaming/PT)
        absolute=False → relative (classic PR path; will temporarily exit PT)
        """
        target_az = _clip(float(az_gim), AZ_MIN, AZ_MAX)
        target_el = _clip(float(el_gim), EL_MIN, EL_MAX)

        if self.sim:
            if absolute:
                self.curr_az, self.curr_el = target_az, target_el
            else:
                self.curr_az = _clip(self.curr_az + target_az, AZ_MIN, AZ_MAX)
                self.curr_el = _clip(self.curr_el + target_el, EL_MIN, EL_MAX)
            self.curr_pos = [self.curr_az, self.curr_el, 0.0]
            return

        if absolute:
            curr_cnt_x = int(round(self.curr_az * self.cnt_az))
            curr_cnt_y = int(round(self.curr_el * self.cnt_el))
            tgt_cnt_x  = int(round(target_az * self.cnt_az))
            tgt_cnt_y  = int(round(target_el * self.cnt_el))

            if (tgt_cnt_x == curr_cnt_x) and (tgt_cnt_y == curr_cnt_y):
                return

            # Streaming path: PA only (PT), no BG -> prevents TC:7 during frequent updates
            self._send_absolute_counts(tgt_cnt_x, tgt_cnt_y, wait=wait)

            self.curr_az, self.curr_el = target_az, target_el

        else:
            # Relative steering: temporarily exit PT to PR+BG, then re-enter PT
            dcnt_x = int(round(target_az * self.cnt_az))
            dcnt_y = int(round(target_el * self.cnt_el))

            was_pt = self.streaming
            if was_pt:
                self._exit_pt()

            if self._send_relative_counts(dcnt_x, dcnt_y, wait=wait):
                self.curr_az = _clip(self.curr_az + (dcnt_x / self.cnt_az), AZ_MIN, AZ_MAX)
                self.curr_el = _clip(self.curr_el + (dcnt_y / self.cnt_el), EL_MIN, EL_MAX)

            if was_pt:
                self._enter_pt()

        self.curr_pos = [self.curr_az, self.curr_el, 0.0]

    # ---------------- UTILITY ----------------
    def stop(self):
        if not self.sim:
            try:
                self._cmd("ST", quiet=True)
            except Exception:
                pass
        print("[STOP] Motion halted.")

    def go_home(self, wait=True):
        """Return to (AZ=0, EL=0 -> Up)."""
        self.move_absolute(0.0, 90.0, wait=wait)

    def wait(self, t=0.1):
        time.sleep(t)
