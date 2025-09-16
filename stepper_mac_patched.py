#pip install pyfirmata keyboard

import os
import logging
import math
from datetime import datetime

from pyfirmata import Arduino, util 
import time
import keyboard
import sys
import termios
from pynput import keyboard
import json
import atexit
import signal
from logging.handlers import RotatingFileHandler
from dataclasses import dataclass, asdict
from typing import Optional, Tuple, List
from math import sin, asin, radians, degrees
from time import time as _time
from time import sleep as _sleep
try:
    from cal_store import load_cal, save_cal, CalData
except Exception:
    # cal_store may not exist yet at import time; we will guard usage
    load_cal = save_cal = CalData = None



#windows
# try:
#     board = Arduino('COM10')  # Set your Arduino port here
# except Exception as e:
#     print(f"Arduino initialization failed: {e}")
#     sys.exit(1)

#mac, open terminal and type this for com number ls /dev/tty.*
try:
    board = Arduino('/dev/cu.usbmodem1101')
except Exception as e:
    print(f"Arduino initialization failed: {e}")
    sys.exit(1)

pins = [11, 10, 9, 8]  # IN1–IN4 pins on ULN2003

#first optical switch 
optical_pin = 7
#second optical switch
optical_pin2 = 5

# Start the iterator to receive input from the board
it = util.Iterator(board)
it.start()

# Set up pin 7 as input
board.digital[optical_pin].mode = 0  # 0 means INPUT
#Set up pin 5 as second input
board.digital[optical_pin2].mode = 0 

#switch 1 is for the smaller disk
switch1 = board.digital[optical_pin].read()
#switch 2 is for the larger worm gear 
switch2 = board.digital[optical_pin2].read()
#use to initialize an array to track the switch states 
last_states = ["", ""]


# attempting directional bias & backlash compensation
DIR_REVERSE_SCALE = 1.0      # multiply reverse steps by this (steady-state)
BACKLASH_STEPS = 0           # extra steps to take up slack when changing direction
_last_move_dir = None        # track direction to detect direction changes
_rev_scale_err_accum = 0.0   # fractional accumulator for reverse scaling

# used to monitor steps and revolutions of the small disk in revs
step_count = 0 
rev_count = 0 
steps_till_rev = 0
steps_since_rev = 0

 #used this varaible to store the steps takesn untill the first complete discrete rev is done, otherwise these steps are lost
# revs + delts steps only account for the steps taken AFTER the revs, this varibale will help log the steps before the first rev
is_1rev_completed = False

last_switch1_state = None
last_switch2_state = None
transition_sequence = []

# determined by calibration
steps_per_rev = 100
# it takes 205 revs + 40 steps for one full gear rotations.
revs_per_rotation = 180 
delta_steps = 40

#use these variable to set a home. 
home_steps = 0
home_revs = 0
home_delta_steps =0 
home_pre_rev_steps =0 
#boolean to track home
home = False

#Edge-driven rev counting for optical switch 1, small disk
rev_count = 0                # net revs from rising edges
pre_rev_steps = 0            # signed: steps from zero to the FIRST rising edge seen after arming
post_rev_steps = 0           # unsigned: steps since LAST rising edge
_last_s1_open = None         # last boolean state of S1 (True=open, False=blocked)
_first_rising_captured = False
_last_rising_stepcount = 0   # step_count at last rising edge
_steps_since_any_edge = 0    # edge holdoff counter to avoid double-trigger at boundary
EDGE_HOLDOFF_STEPS = 6       # min steps between edges to consider another edge valid


#Homing config
HOME_OFFSET_DIR = {+1: 15, -1: 15}  # steps from S2 center toward the approach_dir to exact home
FAST_STEP_DELAY   = 0.001
EDGE_STEP_DELAY   = 0.004
CENTER_STEP_DELAY = 0.006
TWEAK_STEP_DELAY  = 0.008
MAX_FIND_STEPS    = 20000       # hard cap so loops can't run forever
TWEAK_RANGE       = 60           # +/- small steps to satisfy "both open"
FAR_STEP_THRESHOLD = 200   # steps to consider "far" for fast pre-roll

# --- Logging & persistence paths ---
LOG_DIR   = "logs"
LOG_FILE  = os.path.join(LOG_DIR, "stepper.log")  # single, ever-growing log
STATE_DIR = "state"
STEP_FILE = os.path.join(STATE_DIR, "step_count.txt")  # holds only the step_count number

#optical-home offset persistence
OPTICAL_OFFSET_FILE = os.path.join(STATE_DIR, "optical_home_offset.txt")
OPTICAL_HOME_OFFSET_STEPS = 0  # +CW, -CCW (relative to Disk Home)


# Grating geometry
GRATING_D_MM = 1.0 / 1200.0   # groove spacing in mm for 1200 lines/mm

# S2 rotation calibration storage
S2_STEPS_PER_REV = 17971  # measured later; fallback computed if missing
STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0     # derived (S2_STEPS_PER_REV / 360)

# Fallback (from your notes: 205 revs + 40 steps for one S2 360°)
#   revs_per_rotation = motor-shaft revs (S1) per one S2 revolution
#   steps_per_rev      = motor steps per one S1 revolution
def _s2_steps_fallback():
    return revs_per_rotation * steps_per_rev + delta_steps


# Define your step sequence (half-step example)
seq = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

step_delay = 0.001  # 01ms (adjust for your setup)
steps_per_move = 512  # number of steps per move (adjust as needed)

def wavelength_to_theta_deg(lambda_nm, order_m, d_mm=GRATING_D_MM):
    """
    Uses your relation: m * λ = d * sin(theta)
    λ enters in nm; convert to mm. Returns theta in degrees.
    """
    lam_mm = lambda_nm * 1e-6  # 1 nm = 1e-6 mm
    #arg = (order_m * lam_mm) / d_mm.    # non littrow
    arg = (order_m * lam_mm) / (2*d_mm)  # for Littrow
    if abs(arg) > 1.0:
        raise ValueError(f"Unreachable angle: |m*λ/d| = {arg:.6f} > 1")
    theta_rad = math.asin(arg)
    return math.degrees(theta_rad)

def calibrate_s2_steps_per_rev(approach_dir=+1):
    """
    Index to S2 BLOCKED->OPEN edge, then count steps to see it again
    in the same direction. Stores S2_STEPS_PER_REV and STEPS_PER_DEG.
    """
    global S2_STEPS_PER_REV, STEPS_PER_DEG

    # 1) Land on B->O edge (direction-aware)
    _step_until_state(optical_pin2, True,  approach_dir, EDGE_STEP_DELAY)   # ensure BLOCKED
    _step_until_state(optical_pin2, False, approach_dir, EDGE_STEP_DELAY)   # first OPEN after BLOCKED

    # 2) Count to the next identical edge (same direction)
    steps = 0
    # First leave OPEN (go to BLOCKED), so the next OPEN is our next cycle's edge
    steps += _step_until_state(optical_pin2, True, approach_dir, EDGE_STEP_DELAY)
    # Now count from BLOCKED to next OPEN
    steps += _step_until_state(optical_pin2, False, approach_dir, EDGE_STEP_DELAY)

    # 3) Save
    S2_STEPS_PER_REV = steps
    STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0
    print(f"[Cal] S2_STEPS_PER_REV = {S2_STEPS_PER_REV}, STEPS_PER_DEG = {STEPS_PER_DEG:.6f}")
    return S2_STEPS_PER_REV

FAR_STEP_THRESHOLD_MOVE = 600   # if farther than this, sprint at FAST_STEP_DELAY

def _ensure_steps_per_deg():
    global S2_STEPS_PER_REV, STEPS_PER_DEG
    if S2_STEPS_PER_REV is None:
        # Use fallback if not calibrated yet
        S2_STEPS_PER_REV = _s2_steps_fallback()
        STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0
        print(f"[Warn] Using fallback S2_STEPS_PER_REV={S2_STEPS_PER_REV} -> STEPS_PER_DEG={STEPS_PER_DEG:.6f}")

def angle_to_steps(theta_deg: float) -> int:
    """Convert desired grating angle (deg) to motor steps using calibrated steps/deg.
    Positive theta means CCW if GRATING_POSITIVE_CCW=True.
    """
    return steps_from_theta(theta_deg)

def go_home2(approach_dir: int = +1) -> dict:
    """Index on S2 and center the OPEN window, finishing in approach_dir.
    Expects functions: read_s2_state(), _step_once(dir), _move_steps(dir, n), persist_step_count()
    """
    read_s2 = globals().get("read_s2_state", lambda: False)
    step_once = globals().get("_step_once") or globals().get("_step")
    move_steps = globals().get("_move_steps")
    if not (step_once and move_steps):
        raise RuntimeError("Missing _step_once/_step or _move_steps for homing")

    def _seek_s2_edge(direction: int) -> int:
        prev = read_s2()
        steps = 0
        for _ in range(int(S2_STEPS_PER_REV) * 3):
            step_once(direction)
            steps += 1
            cur = read_s2()
            if cur != prev:
                return steps
        raise RuntimeError("S2 edge not found during homing")

    def _measure_open_window(direction: int) -> int:
        if not read_s2():
            _seek_s2_edge(direction)
        steps = 0
        for _ in range(int(S2_STEPS_PER_REV)):
            step_once(direction)
            steps += 1
            if not read_s2():
                return steps
        raise RuntimeError("Open window too large or S2 stuck")

    _seek_s2_edge(approach_dir)
    open_width = _measure_open_window(approach_dir)
    edge_holdoff = max(3, int(0.10 * open_width))

    half = open_width // 2
    move_steps(approach_dir, half)

    try:
        off = int(HOME_OFFSET_DIR.get(str(approach_dir), 0))
    except Exception:
        off = 0
    if off < 0:
        off = int(S2_STEPS_PER_REV) + off
    if off:
        move_steps(approach_dir, off)

    globals()["EDGE_HOLDOFF_STEPS"] = edge_holdoff

    try:
        log_json("home", open_width=open_width, edge_holdoff=edge_holdoff, approach_dir=approach_dir)
    except Exception:
        pass
    try:
        persist_step_count = globals().get("persist_step_count")
        if persist_step_count:
            persist_step_count()
    except Exception:
        pass

    return {"open_width": open_width, "edge_holdoff": edge_holdoff}

class _QuitToMain(Exception):
    pass

def _prompt_or_quit(prompt: str) -> str:
    """
    Read a line; if the user typed 'q' (any case), throw _QuitToMain to unwind back to main menu.
    """
    s = input(prompt).strip()
    if s.lower() == 'q':
        raise _QuitToMain
    return s

# If True: when user chooses "Home-first" while already in S2 OPEN window, auto-switch to 'shortest' (no rehome).
# If False: do gentle rehome-in-window using go_home2() (recommended, repeatable).
AUTO_SHORT_WHEN_AT_HOME = True

def _ask_approach_mode():
    """
    Returns either 'from_home_ccw' (home-first) or 'shortest'.
    'q' at any time returns to main menu.
    If already within S2 OPEN window and user picks Home-first:
      - Either auto-switch to shortest (if AUTO_SHORT_WHEN_AT_HOME=True), or
      - Keep 'from_home_ccw' and rely on go_home2(), which will do a gentle rehome-in-window.
    """
    while True:
        try:
            m = _prompt_or_quit("Approach mode: [H] from home (repeatable) or [S] shortest path? (q=quit) ")
        except _QuitToMain:
            # bubble out to caller; caller should catch and return to main menu
            raise

        m = m.strip().lower()
        if m in ("h", "s"):
            # If they chose Home-first but we're already in S2 OPEN, decide behavior.
            if m == "h":
                s2_is_open = (not _read_blocked(optical_pin2))
                if s2_is_open:
                    if AUTO_SHORT_WHEN_AT_HOME:
                        print("[Info] Already in S2 OPEN window → using shortest path instead of rehoming.")
                        return "shortest"
                    else:
                        print("[Info] Already in S2 OPEN window → will do a gentle rehome-in-window.")
                        return "from_home_ccw"
                # not open → normal home-first
                return "from_home_ccw"
            else:
                return "shortest"

        print("Please enter H, S, or q.")

def _persist_optical_offset():
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = OPTICAL_OFFSET_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(str(OPTICAL_HOME_OFFSET_STEPS))
    os.replace(tmp, OPTICAL_OFFSET_FILE)

def _load_optical_offset():
    global OPTICAL_HOME_OFFSET_STEPS
    try:
        with open(OPTICAL_OFFSET_FILE, "r") as f:
            OPTICAL_HOME_OFFSET_STEPS = int(f.read().strip())
            print(f"[Persist] Optical-home offset = {OPTICAL_HOME_OFFSET_STEPS} steps (from {OPTICAL_OFFSET_FILE})")
    except FileNotFoundError:
        print("[Persist] No optical-home offset file; defaulting to 0")
        OPTICAL_HOME_OFFSET_STEPS = 0
    except Exception as e:
        print(f"[Persist] Could not read {OPTICAL_OFFSET_FILE} ({e}); defaulting to 0")
        OPTICAL_HOME_OFFSET_STEPS = 0


def goto_menu():
    """
    Repeatedly prompt for a signed step move.
    +N => CW, -N => CCW (matches internal convention: direction=+1 is CW).
    Type 'j' to jump to Jog, 'q' to quit back to main.
    """
    global step_delay  # for jog_mode2

    print("\n--- GoTo (relative steps) ---")
    print("Enter an integer step count:")
    print("  +N = CW (forward)")
    print("  -N = CCW (reverse)")
    print("  j  = jump to Jog")
    print("  q  = quit to main menu")

    while True:
        s = input("GoTo steps (+CW / -CCW, j, q): ").strip().lower()
        if s == "q":
            print("Leaving GoTo.")
            return
        if s == "j":
            jog_mode2(step_delay)   # returns here when you quit Jog
            continue

        try:
            user_steps = int(s)
        except ValueError:
            print("Please enter an integer (e.g., 120 or -350), or j, or q.")
            continue

        if user_steps == 0:
            print("Zero steps — nothing to do.")
            continue

        # No sign flip: +N means CW (direction=+1), -N means CCW (direction=-1)
        print(f"[GoTo] Request: {user_steps} steps "
              f"({ 'CW' if user_steps>0 else 'CCW' })")
        _stage_move_signed(user_steps)
        print(f"Done. Current step_count = {step_count}")


def wait_for_initial_sensor_states():
    global switch1, switch2
    while switch1 is None or switch2 is None:
        switch1 = board.digital[optical_pin].read()
        switch2 = board.digital[optical_pin2].read()
        time.sleep(0.01)  # Wait briefly

def setup_logging():
    os.makedirs(LOG_DIR, exist_ok=True)
    # Reset root logger to avoid duplicate handlers after restarts
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    fmt = logging.Formatter('%(asctime)s - %(message)s')

    fh = logging.FileHandler(LOG_FILE, mode='a')   # append to one file
    fh.setFormatter(fmt)
    ch = logging.StreamHandler()
    ch.setFormatter(fmt)

    logger.addHandler(fh)
    logger.addHandler(ch)

    logging.info(f"Logging to {LOG_FILE}")
    return LOG_FILE

def _persist_step_count():
    """Atomically write current step_count to STEP_FILE."""
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = STEP_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(str(step_count))
    os.replace(tmp, STEP_FILE)  # atomic on POSIX/macOS/Windows NTFS

def _load_persisted_step_count():
    """Initialize step_count from STEP_FILE if present."""
    global step_count
    try:
        with open(STEP_FILE, "r") as f:
            txt = f.read().strip()
            step_count = int(txt)
            print(f"[Persist] Restored step_count={step_count} from {STEP_FILE}")
    except FileNotFoundError:
        print("[Persist] No previous step_count; starting at 0")
        step_count = 0
    except Exception as e:
        print(f"[Persist] Could not read {STEP_FILE} ({e}); starting at 0")
        step_count = 0


def main():
    global step_delay

    wait_for_initial_sensor_states()

    # Logging first (so restore message is logged to console/file)
    log_file = setup_logging()

    # Restore last position (if any)
    _load_persisted_step_count()

    # NEW: restore optical-home offset
    _load_optical_offset()


    logging.info("Starting stepper motor control")

    print("Stepper Motor Control")
    print("G: GoTo Step. +N Forward CW, -N Reverse CCW")
    print("J: Jog Mode (UP/DOWN arrows)")
    print("S: Speed Settings")
    print("H: Home Menu")
    print("C: Calibration")
    print("P: Position Menu (Angle/Wavelength)")
    print("Q: Quit")

    while True:
        cmd = input("Enter option GoTo, Jog, Speed Settings, Home, Calibration, Position Menu, Quit (G/J/S/H/C/P/Q): ").strip().upper()
        if cmd in ('G','g'):
           goto_menu()
           flush_input()
        elif cmd in ('J', 'j'):
            #jog_mode(step_delay)
            # for mac, jog2 function uses pynput instead of keyboard package
            jog_mode2(step_delay)
            flush_input()
        elif cmd in ('S', 's'):
            step_delay = set_speed(step_delay)
            flush_input()

        elif cmd in ('H','h'): 
            home_menu()
        elif cmd in ('C','c'):
            print("Starting Calibration")
            calibration()
        elif cmd in ('P','p'):  
            position_menu()
            flush_input()

            #move_to_wavelength(532, 1, mode="from_home_ccw")   # green, 1st order, repeatable approach
            # or
            #move_to_wavelength(650, 1, mode="shortest")        # quicker, chooses direction automatically
            #move_to_angle_deg(12.5, mode="from_home_ccw").     #going to 12.5 degrees, counterclockwise from home

        elif cmd in ('q', 'Q'):
            print("Quitting.")
            flush_input()
            break
        else:
            print("Invalid option. Enter G, J, S, H, C, P, or Q.")
            flush_input()
    for pin in pins:
        board.digital[pin].write(0)
    board.exit()

if __name__ == '__main__':
    main()

# ---- Rotating logger setup ----
_LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_LOG_PATH = os.path.join(_LOG_DIR, "stepper.log")
_logger = logging.getLogger("stepper")
if not _logger.handlers:
    _logger.setLevel(logging.INFO)
    _handler = RotatingFileHandler(_LOG_PATH, maxBytes=5_000_000, backupCount=5)
    _handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    _logger.addHandler(_handler)

def log_json(event: str, **fields):
    try:
        rec = {"event": event, "ts": _time(), **fields}
        _logger.info(json.dumps(rec))
    except Exception:
        _logger.exception("log_json failed")

# ---- Calibration globals ----
try:
    _cal = load_cal() if load_cal else None
except Exception:
    _cal = None

# A safe default if calibration hasn't run yet; will be overwritten after calibration
S2_STEPS_PER_REV = None
STEPS_PER_DEG = None
HOME_OFFSET_DIR = {"+1": 0, "-1": 0}
BACKLASH_STEPS = 0
DIR_REVERSE_SCALE = 1.0

if _cal:
    if getattr(_cal, "S2_STEPS_PER_REV", None):
        S2_STEPS_PER_REV = float(_cal.S2_STEPS_PER_REV)
        STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0
    if getattr(_cal, "HOME_OFFSET_DIR", None):
        HOME_OFFSET_DIR = {str(k): int(v) for k, v in _cal.HOME_OFFSET_DIR.items()}
    if getattr(_cal, "BACKLASH_STEPS", None):
        BACKLASH_STEPS = int(_cal.BACKLASH_STEPS or 0)
    if getattr(_cal, "DIR_REVERSE_SCALE", None):
        DIR_REVERSE_SCALE = float(_cal.DIR_REVERSE_SCALE or 1.0)

if S2_STEPS_PER_REV is None:
    S2_STEPS_PER_REV = 18000.0
if STEPS_PER_DEG is None:
    STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0

EDGE_HOLDOFF_STEPS = 6
PREF_DIR = +1

# ---- Angle/steps conversion (explicit sign) ----
GRATING_POSITIVE_CCW = True
_SIGN = +1 if GRATING_POSITIVE_CCW else -1

def steps_from_theta(theta_deg: float) -> int:
    return int(round(theta_deg * STEPS_PER_DEG * _SIGN))

def theta_from_steps(steps: int) -> float:
    return steps / (STEPS_PER_DEG * _SIGN)

def _stage_move_signed(total_steps: int, direction: int) -> None:
    """Chunked move with periodic drift checks and auto reindex.
    Expects: _move_steps(dir, n), check_step_integrity(tolerance_steps), read_s2_state, _step_once
    """
    if total_steps <= 0:
        return
    move_steps = globals().get("_move_steps")
    check_step_integrity = globals().get("check_step_integrity", None)
    read_s2 = globals().get("read_s2_state", None)
    step_once = globals().get("_step_once") or globals().get("_step")
    if not move_steps or not step_once:
        raise RuntimeError("Missing _move_steps/_step_once for stage move")

    DRIFT_TOL_STEPS = 4
    REINDEX_SPACING = 0.25  # fraction of an S2 revolution

    remaining = abs(int(total_steps))
    moved = 0
    chunk = max(100, int(0.05 * S2_STEPS_PER_REV))
    next_reindex_at = int(REINDEX_SPACING * S2_STEPS_PER_REV)

    def _reindex_nearby_s2(direction: int):
        back = max(20, int(0.02 * S2_STEPS_PER_REV))
        move_steps(-direction, back)
        prev = read_s2()
        for _ in range(int(S2_STEPS_PER_REV) * 3):
            step_once(direction)
            cur = read_s2()
            if cur != prev:
                break
            prev = cur
        # ensure in OPEN then count open width
        ow = 0
        if not read_s2():
            p2 = read_s2()
            for _ in range(int(S2_STEPS_PER_REV)):
                step_once(direction)
                if read_s2() != p2:
                    break
        for _ in range(int(S2_STEPS_PER_REV)):
            step_once(direction)
            ow += 1
            if not read_s2():
                break
        move_steps(direction, ow // 2)
        globals()["EDGE_HOLDOFF_STEPS"] = max(3, int(0.10 * max(1, ow)))

    while remaining > 0:
        take = min(chunk, remaining)
        move_steps(direction, take)
        moved += take
        remaining -= take

        if check_step_integrity:
            try:
                ok, err = check_step_integrity(tolerance_steps=DRIFT_TOL_STEPS)
                if not ok:
                    _reindex_nearby_s2(direction)
                    next_reindex_at = moved + int(REINDEX_SPACING * S2_STEPS_PER_REV)
            except Exception:
                pass

        if moved >= next_reindex_at:
            _reindex_nearby_s2(direction)
            next_reindex_at = moved + int(REINDEX_SPACING * S2_STEPS_PER_REV)

    try:
        persist_step_count = globals().get("persist_step_count")
        if persist_step_count:
            persist_step_count()
    except Exception:
        pass

def move_to_wavelength(lam_nm: float, order: int = 1):
    """Move to a wavelength using calibrated model if available; wraps for final approach dir."""
    go_home = globals().get("go_home2")
    get_step_count = globals().get("get_step_count", lambda: 0)
    stage_move = globals().get("_stage_move_signed")

    theta = None
    try:
        from cal_store import load_cal
        cal = load_cal()
        m = getattr(cal, "WL_MODEL", None)
        if m and isinstance(m, dict):
            ttype = m.get("type")
            if ttype == "affine":
                A, B = float(m["A"]), float(m["B"])
                theta = (lam_nm - A) / B
            elif ttype == "shifted_littrow":
                a, b, t0 = float(m["a"]), float(m["b"]), float(m["t0"])
                x = (lam_nm - a) / b
                x = max(-1.0, min(1.0, x))
                from math import asin, degrees
                theta = degrees(asin(x)) - t0
    except Exception:
        theta = None

    if theta is None:
        d_nm = 1e6 / 1200.0
        val = (order * lam_nm) / (2.0 * d_nm)
        from math import asin, degrees
        theta = degrees(asin(max(-1.0, min(1.0, val))))

    target_steps = steps_from_theta(theta)

    cur = int(get_step_count())
    delta = target_steps - cur
    if (delta > 0 and PREF_DIR < 0) or (delta < 0 and PREF_DIR > 0):
        delta += int(S2_STEPS_PER_REV) * (1 if PREF_DIR > 0 else -1)

    if go_home:
        go_home(approach_dir=PREF_DIR)
    stage_move(abs(int(delta)), PREF_DIR)

# ---- Safe shutdown: persist step_count on exit/signals ----
def _persist_state_on_exit():
    try:
        persist_step_count = globals().get("persist_step_count")
        if persist_step_count:
            persist_step_count()
    except Exception:
        pass

def _handle_signal(signum, frame):
    try:
        _persist_state_on_exit()
    finally:
        raise SystemExit(0)

try:
    import atexit, signal
    atexit.register(_persist_state_on_exit)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _handle_signal)
except Exception:
    pass
