#py -m pip install pyfirmata pynput pyserial
# (optional) py -m pip install keyboard

#windows may need this in terminal: setx ARDUINO_PORT COM5


import builtins
import os, sys, time, math, logging, platform, json
import threading
from collections import deque
from datetime import datetime
from pyfirmata import Arduino, util

try:
    import keyboard as kb  # optionals
except Exception:
    kb = None

try:
    from pynput import keyboard as pynput_keyboard
except Exception as e:
    print("Failed to import pynput.keyboard. Install with: pip install pynput")
    raise

# used to setup logging and control log file size
from log_utils import TailTruncatingFileHandler, enforce_log_size_limit

# Serial port discovery (pyserial comes with pyfirmata, but import tools explicitly)
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None  # we’ll still try known defaults if tools aren’t present

# # Cross-platform input flushing
# if platform.system() == "Windows":
#     # No termios on Windows; define a no-op
#     def flush_input():
#         # Windows console input isn’t line-buffered the same way; nothing to flush.
#         return
# else:
#     import termios
#     def flush_input():
#         # POSIX: flush stdin
#         termios.tcflush(sys.stdin, termios.TCIFLUSH)

def drain_stdin():
    """remove any queued characters so next input() is clean"""
    if platform.system() == "Windows":
        try:
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getwch()
        except Exception:
            pass
    else:
        try:
            import termios
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except Exception:
            pass

INPUT_SETTLE_DELAY_S = 0.05

def settle_terminal_input(delay=INPUT_SETTLE_DELAY_S):
    """Give listener-delivered keystrokes time to land, then flush stdin."""
    try:
        time.sleep(max(0.0, float(delay)))
    finally:
        drain_stdin()

def clean_input(prompt=""):
    drain_stdin()
    return builtins.input(prompt)

input = clean_input

def prompt_cmd(prompt):
    drain_stdin()
    return input(prompt).strip().upper()

def _make_keyboard_listener(*, on_press=None, on_release=None):
    try:
        return pynput_keyboard.Listener(on_press=on_press, on_release=on_release, suppress=True)
    except TypeError:
        return pynput_keyboard.Listener(on_press=on_press, on_release=on_release)


def _pick_arduino_port(preferred=None):
    """
    Return a serial port string for the Arduino on macOS or Windows.
    - If 'preferred' is provided, try it first.
    - Otherwise scan available ports and pick the most Arduino-looking one.
    """
    # If caller passes an explicit port, try that first
    if preferred:
        return preferred

    # If we can't scan, fall back to sensible defaults
    if list_ports is None:
        return "COM10" if platform.system() == "Windows" else "/dev/cu.usbmodem1101"

    candidates = list(list_ports.comports())
    if not candidates:
        # No ports found; fall back
        return "COM10" if platform.system() == "Windows" else "/dev/cu.usbmodem101"

    # Heuristics: prefer ports whose descriptions mention Arduino/USB/Modem
    def score(p):
        desc = f"{p.description or ''} {p.manufacturer or ''}".lower()
        s = 0
        if "arduino" in desc: s += 10
        if "usb" in desc:     s += 3
        if "modem" in desc:   s += 2
        if platform.system() == "Windows" and p.device.upper().startswith("COM"):
            s += 1
        if platform.system() != "Windows" and "/dev/cu." in p.device:
            s += 1
        return s

    best = max(candidates, key=score)
    return best.device

# --- Board init (cross-platform, lazy for GUI safety) ---
PORT_OVERRIDE = os.getenv("ARDUINO_PORT")  # allow manual override via env var
port = None
board = None
it = None

pins = [11, 10, 9, 8]  # IN1–IN4 pins on ULN2003

#first optical switch 
optical_pin = 7
#second optical switch
optical_pin2 = 5

switch1 = None
switch2 = None
# use to initialize an array to track the switch states
last_states = [None, None]

def _init_board():
    global port, board, it, switch1, switch2, last_states
    if board is not None:
        return board

    port = _pick_arduino_port(PORT_OVERRIDE)
    try:
        board = Arduino(port)
    except Exception as e:
        raise RuntimeError(
            f"Arduino initialization failed on port '{port}': {e}\n"
            "Tips:\n"
            "  - Check the correct COM port (Windows) or /dev/cu.* (macOS).\n"
            "  - Try setting ARDUINO_PORT, e.g. ARDUINO_PORT=COM5\n"
            "  - Ensure drivers/permissions are OK."
        ) from e

    it = util.Iterator(board)
    it.start()
    time.sleep(1)

    board.digital[optical_pin].mode = 0
    board.digital[optical_pin2].mode = 0
    switch1 = board.digital[optical_pin].read()
    switch2 = board.digital[optical_pin2].read()
    last_states = [None, None]
    return board


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

# Legacy controller logic treated one software "step" as a full 8-state sequence cycle.
# We now count each electrical half-step directly, so old hand-tuned motion distances
# need to be scaled by this factor to preserve the same physical travel.
LEGACY_STEP_SCALE = 8

def _legacy_steps(n):
    return int(round(float(n) * LEGACY_STEP_SCALE))

# Hardware/sign convention:
#   stage CW  = positive
#   stage CCW = negative
# The current ULN2003 sequence order is treated as CW-positive.
MOTOR_SEQUENCE_SIGN = +1

# determined by calibration; 0.9° motor => ~800 half-steps/rev
steps_per_rev = 800
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
EDGE_HOLDOFF_STEPS = _legacy_steps(6)       # min half-steps between edges to consider another edge valid


#Homing config
HOME_DEFAULT_APPROACH_DIR = +1
HOME_OFFSET_DIR = {+1: 0, -1: 0}  # additional half-steps from S2 center toward exact home; keep zero unless measured
FAST_STEP_DELAY   = 0.001
EDGE_STEP_DELAY   = 0.004
CENTER_STEP_DELAY = 0.006
TWEAK_STEP_DELAY  = 0.008
MAX_FIND_STEPS    = _legacy_steps(20000)       # hard cap so loops can't run forever
TWEAK_RANGE       = _legacy_steps(100)         # +/- bounded half-step search to satisfy "both open"
FAR_STEP_THRESHOLD = _legacy_steps(200)        # half-steps to consider "far" for fast pre-roll
REHOME_BUMP_STEPS  = _legacy_steps(100)        # half-steps to leave the OPEN window when already at home
FIXED_FINAL_APPROACH_DIR = +1    # tune this to the preferred final arrival direction on S2
FIXED_FINAL_APPROACH_BACKOFF_STEPS = _legacy_steps(30)  # should exceed S2 backlash for shortest-path moves
PERSIST_EVERY_STEPS = _legacy_steps(10)  # persist at least every legacy-10-step equivalent during long moves
PERSIST_EVERY_SECONDS = 0.25
_last_persist_step = 0
_last_persist_time = 0.0
DEBUG_MOTION_LOGGING = False     # True -> log sensor snapshot on every monitor_sensors() call
DEBUG_TERMINAL_OUTPUT = False    # True -> print sensor snapshot on every monitor_sensors() call
JOG_STREAM_SENSORS = False       # True while jog mode is active
_last_sensor_terminal_width = 0
_last_sensor_terminal_time = 0.0
JOG_DISPLAY_DIR = 0
JOG_TERMINAL_REFRESH_S = 0.05
OFFSET_ADJUST_REFRESH_S = 0.05
OFFSET_ADJUST_MAX_PENDING = 8
LOG_TO_CONSOLE = False
MIN_STEP_DELAY_MS = 1
MAX_STEP_DELAY_MS = 1000
JOG_SPEED_STEP_MS = 1

# --- Logging & persistence paths ---
LOG_DIR   = "logs"
LOG_FILE  = os.path.join(LOG_DIR, "stepper.log")  # single, ever-growing log
STATE_DIR = "state"
STEP_FILE = os.path.join(STATE_DIR, "step_count.txt")  # holds only the step_count number
JOG_SPEED_FILE = os.path.join(STATE_DIR, "jog_speed_ms.txt")

#optical-home offset persistence
OPTICAL_OFFSET_FILE = os.path.join(STATE_DIR, "optical_home_offset.txt")
OPTICAL_HOME_OFFSET_STEPS = 0  # +CW, -CCW (relative to Disk Home)

# Grating geometry
GRATING_D_MM = 1.0 / 1200.0   # groove spacing in mm for 1200 lines/mm


# Half-step drive sequence. Each row is one electrical half-step.
seq = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
]
_seq_index = 0

# S2 rotation calibration storage
S2_STEPS_PER_REV = 18000 * len(seq)  # legacy controller counted 1 "step" per full sequence cycle
STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0     # derived (S2_STEPS_PER_REV / 360)

# Fallback (from your notes: 205 revs + 40 steps for one S2 360°)
#   revs_per_rotation = motor-shaft revs (S1) per one S2 revolution
#   steps_per_rev      = motor steps per one S1 revolution
def _s2_steps_fallback():
    return S2_STEPS_PER_REV

# grabbing calibration data from cal_store.py if available
try:
    from cal_store import (
    save_cal,
    load_cal,
    apply_cal_to_globals,
    load_wl_cal,
    save_wl_cal,
)
except Exception:
    load_cal = save_cal = CalData = None


# One live calibration instance (loaded on startup)
_cal = None
_runtime_initialized = False
_runtime_init_lock = threading.RLock()


step_delay = 0.001  # 01ms (adjust for your setup)
steps_per_move = 512  # number of steps per move (adjust as needed)

def _delay_to_ms(delay_s):
    return int(round(float(delay_s) * 1000.0))

def _clamp_step_delay_ms(ms):
    return max(MIN_STEP_DELAY_MS, min(MAX_STEP_DELAY_MS, int(round(float(ms)))))

def _persist_jog_speed():
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = JOG_SPEED_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(str(_delay_to_ms(step_delay)))
    os.replace(tmp, JOG_SPEED_FILE)

def _set_step_delay_ms(ms, *, persist=True):
    global step_delay
    step_delay = _clamp_step_delay_ms(ms) / 1000.0
    if persist:
        _persist_jog_speed()
    return step_delay

def _load_persisted_jog_speed():
    global step_delay
    try:
        with open(JOG_SPEED_FILE, "r") as f:
            ms = _clamp_step_delay_ms(int(f.read().strip()))
            step_delay = ms / 1000.0
            print(f"[Persist] Jog speed = {ms} ms/step (from {JOG_SPEED_FILE})")
    except FileNotFoundError:
        print(f"[Persist] No jog-speed file; defaulting to {_delay_to_ms(step_delay)} ms/step")
    except Exception as e:
        print(f"[Persist] Could not read {JOG_SPEED_FILE} ({e}); using {_delay_to_ms(step_delay)} ms/step")

def jog_mode2():
    global JOG_STREAM_SENSORS, JOG_DISPLAY_DIR, step_delay
    print("\n--- Jog Mode ---")
    print("Hold UP for forward, DOWN for reverse. Press Q to quit jog mode.")
    print("RIGHT arrow speeds up by 1 ms/step. LEFT arrow slows down by 1 ms/step.")
    print("[Jog] Manual jog uses exact steps only; backlash compensation is disabled here.")
    JOG_STREAM_SENSORS = True
    JOG_DISPLAY_DIR = 0

    running = True
    forward_pressed = False
    reverse_pressed = False
    speed_adjust_queue = deque()
    speed_keys_held = set()

    def _apply_speed_delta(delta_ms):
        old_ms = _delay_to_ms(step_delay)
        _set_step_delay_ms(old_ms + delta_ms, persist=True)
        new_ms = _delay_to_ms(step_delay)
        if new_ms != old_ms:
            _clear_sensor_terminal_line()
            print(f"[Jog] Speed set to {new_ms} ms/step.")

    def on_press(key):
        nonlocal running, forward_pressed, reverse_pressed
        if key == pynput_keyboard.Key.up:
            forward_pressed = True
        elif key == pynput_keyboard.Key.down:
            reverse_pressed = True
        elif key == pynput_keyboard.Key.right:
            if key not in speed_keys_held:
                speed_keys_held.add(key)
                speed_adjust_queue.append(-JOG_SPEED_STEP_MS)
        elif key == pynput_keyboard.Key.left:
            if key not in speed_keys_held:
                speed_keys_held.add(key)
                speed_adjust_queue.append(+JOG_SPEED_STEP_MS)
        elif hasattr(key, 'char') and key.char and key.char.lower() == 'q':
            running = False
            _clear_sensor_terminal_line()
            print("Exiting jog mode.")
            print("step count", step_count)
            print("rev count", rev_count)
            return False

    def on_release(key):
        nonlocal forward_pressed, reverse_pressed
        if key == pynput_keyboard.Key.up:
            forward_pressed = False
        elif key == pynput_keyboard.Key.down:
            reverse_pressed = False
        elif key in (pynput_keyboard.Key.left, pynput_keyboard.Key.right):
            speed_keys_held.discard(key)

    listener = None
    try:
        listener = _make_keyboard_listener(on_press=on_press, on_release=on_release)
        listener.start()
        while running:
            JOG_DISPLAY_DIR = 0
            while speed_adjust_queue:
                _apply_speed_delta(speed_adjust_queue.popleft())
            if forward_pressed:
                JOG_DISPLAY_DIR = +1
                move_exact(steps=1, step_delay=step_delay, direction=1)
                while forward_pressed and running:
                    JOG_DISPLAY_DIR = +1
                    while speed_adjust_queue:
                        _apply_speed_delta(speed_adjust_queue.popleft())
                    move_exact(steps=1, step_delay=step_delay, direction=1)

            if reverse_pressed:
                JOG_DISPLAY_DIR = -1
                move_exact(steps=1, step_delay=step_delay, direction=-1)
                while reverse_pressed and running:
                    JOG_DISPLAY_DIR = -1
                    while speed_adjust_queue:
                        _apply_speed_delta(speed_adjust_queue.popleft())
                    move_exact(steps=1, step_delay=step_delay, direction=-1)

            monitor_sensors(reason="jog")
            time.sleep(0.01)
    finally:
        JOG_STREAM_SENSORS = False
        JOG_DISPLAY_DIR = 0
        _clear_sensor_terminal_line()
        if listener:
            listener.stop()
            listener.join(timeout=0.2)
        settle_terminal_input()


def _start_cancel_listener(hint="Press 'q' or Esc to cancel and return to the main menu."):
    print(hint)
    _set_cancel(False)

    def on_press(key):
        try:
            if key == pynput_keyboard.Key.esc:
                _set_cancel(True); return False
            if hasattr(key, "char") and key.char and key.char.lower() == 'q':
                _set_cancel(True); return False
        except Exception:
            pass

    listener = _make_keyboard_listener(on_press=on_press)
    listener.start()
    return listener


def wavelength_to_theta_deg(lambda_nm, order_m, d_mm=GRATING_D_MM):
    """
    Ideal Littrow relation: m * λ = 2 * d * sin(theta)
    λ enters in nm; convert to mm. Returns theta in degrees.
    """
    lam_mm = lambda_nm * 1e-6  # 1 nm = 1e-6 mm
    arg = (order_m * lam_mm) / (2*d_mm)  # for Littrow
    if abs(arg) > 1.0:
        raise ValueError(f"Unreachable angle: |m*λ/d| = {arg:.6f} > 1")
    theta_rad = math.asin(arg)
    return math.degrees(theta_rad)

def _deg_to_littrow_term(theta_deg, d_mm=GRATING_D_MM):
    """
    Return X = (2*d) * sin(theta) in mm for Littrow (m*λ = 2d sinθ).
    We'll fit λ_nm ≈ a + b * (X_mm * 1e6)  where X is converted to nm.
    """
    x_mm = 2.0 * d_mm * math.sin(math.radians(theta_deg))
    return x_mm * 1e6  # convert mm -> nm

def _fit_littrow_linear(points):
    """
    points: list of dicts with keys (theta_deg, lambda_nm, order_m)
    We reduce to y = λ_nm / m vs. X = (2d sinθ)_nm  and fit λ0 ≈ a + b * X
    Returns (a, b).
    """
    X, Y = [], []
    for p in points:
        m = float(p["order_m"])
        if m == 0:
            # for m=0 order, Littrow gives λ/m undefined; skip or treat as λ≈a (X=0),
            # but we generally use m≠0 references for robust fit.
            continue
        X.append(_deg_to_littrow_term(p["theta_deg"]))
        Y.append(p["lambda_nm"] / m)
    if len(X) < 2:
        raise ValueError("Need at least two nonzero-order references to fit Littrow linear model.")
    n = len(X)
    sumx = sum(X); sumy = sum(Y)
    sumxx = sum(x*x for x in X)
    sumxy = sum(x*y for x,y in zip(X,Y))
    denom = n*sumxx - sumx*sumx
    if abs(denom) < 1e-9:
        raise ValueError("Degenerate Littrow fit (colinear sums).")
    b = (n*sumxy - sumx*sumy) / denom
    a = (sumy - b*sumx) / n
    return a, b

def _fit_littrow_through_origin(points):
    """
    Fit λ/m ≈ b * (2d sinθ)_nm.
    Returns (a, b) with a fixed to 0.
    """
    X, Y = [], []
    for p in points:
        m = float(p["order_m"])
        if m == 0:
            continue
        X.append(_deg_to_littrow_term(p["theta_deg"]))
        Y.append(p["lambda_nm"] / m)
    if len(X) < 1:
        raise ValueError("Need at least one nonzero-order reference for a through-origin fit.")
    sumxx = sum(x*x for x in X)
    sumxy = sum(x*y for x, y in zip(X, Y))
    if abs(sumxx) < 1e-12:
        raise ValueError("Degenerate through-origin fit (sumxx≈0).")
    return 0.0, (sumxy / sumxx)

def _fit_sin_cos_model(points):
    """
    Fit λ/m ≈ A*sinθ + B*cosθ.
    Returns (A_nm, B_nm).
    """
    s2 = c2 = sc = sy = cy = 0.0
    used = 0
    for p in points:
        m = float(p["order_m"])
        if m == 0:
            continue
        theta_rad = math.radians(float(p["theta_deg"]))
        sval = math.sin(theta_rad)
        cval = math.cos(theta_rad)
        yval = float(p["lambda_nm"]) / m
        s2 += sval * sval
        c2 += cval * cval
        sc += sval * cval
        sy += sval * yval
        cy += cval * yval
        used += 1
    if used < 2:
        raise ValueError("Need at least two nonzero-order references to fit a sine/cosine model.")
    denom = s2 * c2 - sc * sc
    if abs(denom) < 1e-12:
        raise ValueError("Degenerate sine/cosine fit (denominator≈0).")
    sin_coeff = (sy * c2 - cy * sc) / denom
    cos_coeff = (cy * s2 - sy * sc) / denom
    return sin_coeff, cos_coeff

def _normalize_wl_model(model):
    if not isinstance(model, dict):
        return {"kind": "affine_littrow", "a": 0.0, "b": 1.0}

    kind = model.get("kind")
    if not kind:
        if "sin_coeff_nm" in model or "cos_coeff_nm" in model:
            kind = "sin_cos"
        elif "amplitude_nm" in model or "theta0_deg" in model:
            kind = "sin_theta_offset"
        else:
            kind = "affine_littrow"

    if kind == "affine_littrow":
        return {
            "kind": kind,
            "a": float(model.get("a", 0.0)),
            "b": float(model.get("b", 1.0)),
        }

    if kind == "sin_theta_offset":
        return {
            "kind": kind,
            "amplitude_nm": float(model.get("amplitude_nm", 0.0)),
            "theta0_deg": float(model.get("theta0_deg", 0.0)),
        }

    if kind == "sin_cos":
        sin_coeff = float(model.get("sin_coeff_nm", model.get("A", 0.0)))
        cos_coeff = float(model.get("cos_coeff_nm", model.get("B", 0.0)))
        amplitude_nm = math.hypot(sin_coeff, cos_coeff)
        phase_deg = math.degrees(math.atan2(cos_coeff, sin_coeff)) if amplitude_nm else 0.0
        return {
            "kind": kind,
            "sin_coeff_nm": sin_coeff,
            "cos_coeff_nm": cos_coeff,
            "amplitude_nm": amplitude_nm,
            "phase_deg": phase_deg,
        }

    return {"kind": "affine_littrow", "a": 0.0, "b": 1.0}

def _wl_model_value(theta_deg, model):
    """
    Evaluate λ/m in nm at the supplied θ, using the saved model family.
    """
    model = _normalize_wl_model(model)
    theta_rad = math.radians(theta_deg)

    if model["kind"] == "affine_littrow":
        return model["a"] + model["b"] * _deg_to_littrow_term(theta_deg)
    if model["kind"] == "sin_theta_offset":
        return model["amplitude_nm"] * math.sin(theta_rad + math.radians(model["theta0_deg"]))
    if model["kind"] == "sin_cos":
        return (
            model["sin_coeff_nm"] * math.sin(theta_rad)
            + model["cos_coeff_nm"] * math.cos(theta_rad)
        )
    raise ValueError(f"Unknown wavelength model kind: {model['kind']}")

def _wl_model_summary(model):
    model = _normalize_wl_model(model)
    if model["kind"] == "affine_littrow":
        return (
            "affine Littrow: "
            f"λ/m ≈ {model['a']:.6f} + {model['b']:.6f}·(2d sinθ)_nm"
        )
    if model["kind"] == "sin_theta_offset":
        return (
            "phase-shift sine: "
            f"λ/m ≈ {model['amplitude_nm']:.6f}·sin(θ + {model['theta0_deg']:.6f}°)"
        )
    if model["kind"] == "sin_cos":
        return (
            "sine/cosine: "
            f"λ/m ≈ {model['sin_coeff_nm']:.6f}·sinθ + {model['cos_coeff_nm']:.6f}·cosθ"
        )
    return "unknown wavelength model"

def _wl_model_residuals(points, model):
    res = []
    for p in points:
        mref = float(p["order_m"])
        if mref == 0:
            continue
        y = float(p["lambda_nm"]) / mref
        yhat = _wl_model_value(float(p["theta_deg"]), model)
        res.append(y - yhat)
    return res


def _theta_deg_from_lambda(lambda_nm, order_m):
    """
    Use the saved wavelength model if available; otherwise fall back to ideal Littrow.
    Supported models:
      - affine Littrow: λ/m ≈ a + b·(2d sinθ)_nm
      - phase-shift sine: λ/m ≈ A·sin(θ + θ0)
      - sine/cosine: λ/m ≈ A·sinθ + B·cosθ
    """
    WL = _wl_load(STATE_DIR)
    model = _normalize_wl_model(WL.get("model"))
    target = lambda_nm / float(order_m)

    if model["kind"] == "affine_littrow":
        target -= model["a"]
        denom = model["b"] * (2.0 * GRATING_D_MM * 1e6)
        if abs(denom) < 1e-12:
            raise ValueError("Invalid wavelength model (b near zero).")
        arg = target / denom
        if abs(arg) > 1.0:
            raise ValueError(f"Requested λ={lambda_nm} nm, m={order_m} not reachable (|sinθ|={arg:.6f}>1).")
        return math.degrees(math.asin(arg))

    if model["kind"] == "sin_theta_offset":
        amplitude = model["amplitude_nm"]
        if abs(amplitude) < 1e-12:
            raise ValueError("Invalid wavelength model (amplitude near zero).")
        arg = target / amplitude
        if abs(arg) > 1.0:
            raise ValueError(f"Requested λ={lambda_nm} nm, m={order_m} not reachable (|sinθ|={arg:.6f}>1).")
        return math.degrees(math.asin(arg)) - model["theta0_deg"]

    if model["kind"] == "sin_cos":
        amplitude = model["amplitude_nm"]
        if abs(amplitude) < 1e-12:
            raise ValueError("Invalid wavelength model (amplitude near zero).")
        arg = target / amplitude
        if abs(arg) > 1.0:
            raise ValueError(f"Requested λ={lambda_nm} nm, m={order_m} not reachable (|sinθ|={arg:.6f}>1).")
        return math.degrees(math.asin(arg)) - model["phase_deg"]

    raise ValueError(f"Unknown wavelength model kind: {model['kind']}")



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

    # Persist into calibration JSON if available
    # if _cal is not None:
    #     _cal.S2_STEPS_PER_REV = int(S2_STEPS_PER_REV)
    #     save_cal(_cal)
   
    path = save_cal(
        STATE_DIR, 
        s2_steps_per_rev=steps, 
        steps_per_deg=steps/360.0
    )

    print(f"[Cal] S2 saved to: {path}")
    return S2_STEPS_PER_REV


FAR_STEP_THRESHOLD_MOVE = _legacy_steps(600)   # if farther than this, sprint at FAST_STEP_DELAY


# cancellimng functions
CANCEL = False

def _set_cancel(v=True):
    global CANCEL
    CANCEL = v

def _check_cancel():
    # Raise to unwind back to menus
    if CANCEL:
        raise _QuitToMain

def _stop_cancel_listener(listener):
    try:
        if listener:
            listener.stop()
            listener.join(timeout=0.2)
    finally:
        _set_cancel(False)
        settle_terminal_input()

def _run_cancellable(fn, *args, **kwargs):
    """
    Start cancel listener, run fn, cleanly stop & release coils on cancel or completion.
    """
    listener = _start_cancel_listener()
    try:
        return fn(*args, **kwargs)
    except _QuitToMain:
        print("[Cancel] Aborted operation; returning to main menu.")
    finally:
        _stop_cancel_listener(listener)
        # release coils so we don't keep holding torque if we bailed out mid-move
        for pin in pins:
            board.digital[pin].write(0)


def _ensure_steps_per_deg():
    global S2_STEPS_PER_REV, STEPS_PER_DEG
    if S2_STEPS_PER_REV is None:
        # Use fallback if not calibrated yet
        S2_STEPS_PER_REV = _s2_steps_fallback()
        STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0
        print(f"[Warn] Using fallback S2_STEPS_PER_REV={S2_STEPS_PER_REV} -> STEPS_PER_DEG={STEPS_PER_DEG:.6f}")

def _upgrade_legacy_calibration_units():
    global steps_per_rev, S2_STEPS_PER_REV, STEPS_PER_DEG, BACKLASH_STEPS, EDGE_HOLDOFF_STEPS

    legacy = False
    if steps_per_rev not in (None, 0) and steps_per_rev < 200:
        legacy = True
    if S2_STEPS_PER_REV not in (None, 0) and S2_STEPS_PER_REV < 50000:
        legacy = True
    if STEPS_PER_DEG not in (None, 0) and STEPS_PER_DEG < 100:
        legacy = True

    if not legacy:
        return False

    notes = []
    if steps_per_rev not in (None, 0) and steps_per_rev < 200:
        steps_per_rev = int(round(steps_per_rev * LEGACY_STEP_SCALE))
        notes.append(f"S1 steps/rev -> {steps_per_rev}")
    if S2_STEPS_PER_REV not in (None, 0) and S2_STEPS_PER_REV < 50000:
        S2_STEPS_PER_REV = int(round(S2_STEPS_PER_REV * LEGACY_STEP_SCALE))
        notes.append(f"S2 steps/rev -> {S2_STEPS_PER_REV}")
    if STEPS_PER_DEG not in (None, 0) and STEPS_PER_DEG < 100:
        STEPS_PER_DEG = float(STEPS_PER_DEG * LEGACY_STEP_SCALE)
        notes.append(f"steps/deg -> {STEPS_PER_DEG:.6f}")
    if BACKLASH_STEPS not in (None, 0):
        BACKLASH_STEPS = int(round(BACKLASH_STEPS * LEGACY_STEP_SCALE))
        notes.append(f"backlash -> {BACKLASH_STEPS}")
    if EDGE_HOLDOFF_STEPS not in (None, 0) and EDGE_HOLDOFF_STEPS < _legacy_steps(3):
        EDGE_HOLDOFF_STEPS = int(round(EDGE_HOLDOFF_STEPS * LEGACY_STEP_SCALE))
        notes.append(f"edge_holdoff -> {EDGE_HOLDOFF_STEPS}")

    print("[Cal] Upgraded legacy calibration values to half-step units:")
    for note in notes:
        print(f"  - {note}")
    print("[Cal] Re-run S1, S2, backlash, and optical-home calibration when convenient.")
    return True

def _steps_to_theta_deg(abs_steps):
    _ensure_steps_per_deg()
    return (float(abs_steps) - OPTICAL_HOME_OFFSET_STEPS) / STEPS_PER_DEG

def _wrap_delta_steps(delta_steps):
    _ensure_steps_per_deg()
    rev = int(round(S2_STEPS_PER_REV)) if S2_STEPS_PER_REV else 0
    delta = int(round(delta_steps))
    if rev <= 0:
        return delta
    half = rev / 2.0
    while delta > half:
        delta -= rev
    while delta < -half:
        delta += rev
    return delta

def _final_approach_backoff_steps():
    return max(int(FIXED_FINAL_APPROACH_BACKOFF_STEPS), int(BACKLASH_STEPS or 0) + 4)

def _move_to_target_steps_fixed_approach(target_steps, final_dir=FIXED_FINAL_APPROACH_DIR):
    """
    Reach the target by the shorter route while making the last segment arrive
    in a consistent direction. This reduces S2 backlash sensitivity in "shortest" mode.
    """
    target_steps = int(round(target_steps))
    current = step_count
    direct_delta = _wrap_delta_steps(target_steps - current)
    if direct_delta == 0:
        return

    if (1 if direct_delta > 0 else -1) == final_dir:
        _stage_move_signed(direct_delta)
        return

    pretarget = target_steps - final_dir * _final_approach_backoff_steps()
    pre_delta = _wrap_delta_steps(pretarget - current)
    if pre_delta != 0:
        _stage_move_signed(pre_delta)

    final_delta = _wrap_delta_steps(target_steps - step_count)
    if final_delta == 0:
        return
    if (1 if final_delta > 0 else -1) != final_dir and S2_STEPS_PER_REV:
        final_delta += final_dir * int(round(S2_STEPS_PER_REV))
    _stage_move_signed(final_delta)

def angle_to_steps(theta_deg):
    _ensure_steps_per_deg()
    direction = 1 #if CW is positive and -1 if CW is negative
    steps = int(round(direction*theta_deg * STEPS_PER_DEG))
    return steps
    

def move_to_angle_deg(theta_deg, mode="from_home_ccw"):
    """
    mode:
      - "from_home_ccw": re-index to Disk Home (repeatable), then move to the
                         DISK-HOME-relative target that already includes the
                         saved optical offset.
      - "shortest": move from current position to the DISK-HOME-relative target
                    (also includes the saved optical offset) via the shorter arc.

    Notes:
      - `OPTICAL_HOME_OFFSET_STEPS` is always included in the target, so θ=0 refers
        to your *optical home* (Disk Home + offset), not raw Disk Home.
      - For better repeatability, the final approach is forced to the same direction
        in `shortest` mode (see `FIXED_FINAL_APPROACH_DIR` / `FIXED_FINAL_APPROACH_BACKOFF_STEPS`).
    """
    _ensure_steps_per_deg()
    target_steps = _target_steps_from_disk_home(theta_deg)  # includes OPTICAL_HOME_OFFSET_STEPS

    if mode == "from_home_ccw":
        # Always begin from a known reference
        if not go_home2():
            print("[Home] Could not establish Disk Home; move aborted.")
            return
        #_approach_abs_steps(target_steps, mode="from_home_ccw")
        _stage_move_signed(target_steps)  

    elif mode == "shortest":
        _move_to_target_steps_fixed_approach(target_steps)

    else:
        raise ValueError("mode must be 'from_home_ccw' or 'shortest'")


def move_to_wavelength_nonlinear(lambda_nm, order_m, mode="from_home_ccw"):
    try:
        theta = _theta_deg_from_lambda(lambda_nm, order_m)
    except ValueError as e:
        print(f"[WL] {e}")
        return
    print(f"Target θ = {theta:.4f}° for λ={lambda_nm} nm, m={order_m}")
    move_to_angle_deg(theta, mode=mode)

# ideal Littrow with no fitted correction
def move_to_wavelength(lambda_nm, order_m, mode="from_home_ccw"):
    theta = wavelength_to_theta_deg(lambda_nm, order_m)
    print(f"Target θ = {theta:.4f}° for λ={lambda_nm} nm, m={order_m}")
    move_to_angle_deg(theta, mode=mode)

def estimate_steps_from_S1():
    """
    Build a second estimate of total steps from S1 information.
    Assumes you keep:
      - rev_count (signed)
      - steps_since_rev (signed, since last rising OPEN edge)
      - pre_rev_steps (signed, from power-up/home to the first rising edge)
    If you don’t trust pre_rev_steps, set it to 0 after home.
    """
    # You can store the sign convention: CCW positive (same as step_count)
    # Here I mirror your sign usage.
    return rev_count * steps_per_rev + steps_since_rev - pre_rev_steps

def check_step_integrity(tolerance_steps=_legacy_steps(4)):
    est = estimate_steps_from_S1()
    err = est - step_count
    if abs(err) > tolerance_steps:
        print(f"[Drift] S1-based est differs from step_count by {err} steps! Re-indexing recommended.")
        return False, err
    return True, err



def move_biased(steps, step_delay, direction):
    # Apply direction-change backlash and steady-state reverse scaling.
    global _last_move_dir, _rev_scale_err_accum
    _check_cancel() 

    # If direction changes, take up backlash in the new direction first.
    if _last_move_dir is not None and direction != _last_move_dir:
        _persist_step_count()
        if BACKLASH_STEPS > 0:
            # Take up slack physically, but do not advance the logical stage coordinate.
            move_stepper(seq, BACKLASH_STEPS, step_delay, direction, count_logical_steps=False)

    extra_reverse_steps = 0
    if direction == -1 and DIR_REVERSE_SCALE != 1.0:
        # Accumulate only the *extra* physical steps needed for reverse compensation.
        # These should not advance the logical stage coordinate.
        _rev_scale_err_accum += steps * (DIR_REVERSE_SCALE - 1.0)
        extra = int(round(_rev_scale_err_accum))
        _rev_scale_err_accum -= extra
        extra_reverse_steps = max(0, extra)

    if steps > 0:
        move_stepper(seq, steps, step_delay, direction, count_logical_steps=True)
    if extra_reverse_steps > 0:
        move_stepper(seq, extra_reverse_steps, step_delay, direction, count_logical_steps=False)
        
    _last_move_dir = direction
    _check_cancel() 

def move_exact(steps, step_delay, direction):
    """Move literal stage steps with no backlash or reverse-scale compensation."""
    global _last_move_dir
    if steps <= 0:
        return
    move_stepper(seq, int(steps), step_delay, direction)
    _last_move_dir = direction

def move_stepper(seq, steps,step_delay, direction, count_logical_steps=True):
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, _steps_since_any_edge
    global home, switch1, switch2, steps_per_rev, steps_till_rev, steps_since_rev, is_1rev_completed
    global _seq_index
    # do NOT reinitialize them here!
    # just update their values as you go
    motor_direction = _motor_direction(direction)

    for _ in range(steps):
        _check_cancel()
        _seq_index = (_seq_index + (1 if motor_direction > 0 else -1)) % len(seq)
        for pin, val in zip(pins, seq[_seq_index]):
            board.digital[pin].write(val)

        # One commanded step has physically happened.
        if count_logical_steps:
            step_count += 1 * direction
            steps_since_rev += 1 * direction
            _steps_since_any_edge += 1 if direction > 0 else -1

        monitor_sensors()

         # Read switch 1 (smaller disk) state each time you move a half-step
        switch1 = board.digital[optical_pin].read()

        if switch1 is None:
            state1 = "WAITING"
        elif switch1:
            state1 = "BLOCKED"
        else:
            state1 = "OPEN"

        if count_logical_steps and state1 == "OPEN":
            is_1rev_completed = True

        # Track transition sequence for open->blocked->open
        if count_logical_steps and last_switch1_state is not None:
            # Only add when state changes
            if state1 != last_switch1_state:
                transition_sequence.append(state1)
                # Keep last three states only
                if len(transition_sequence) > 3:
                    transition_sequence.pop(0)

                # Check for open->blocked->open
                if transition_sequence == ["OPEN", "BLOCKED", "OPEN"] and abs(_steps_since_any_edge) >= EDGE_HOLDOFF_STEPS:
                    rev_count += 1 * direction
                    is_1rev_completed = True
                    steps_since_rev = 0
                    _steps_since_any_edge = 0
                    transition_sequence = ["OPEN"] # reset for next revolution

        last_switch1_state = state1
        time.sleep(step_delay)

        if count_logical_steps:
            _maybe_persist_step_count()

        if count_logical_steps and is_1rev_completed == False and state1 == "BLOCKED":
            steps_till_rev += 1 *direction

    # Turn all pins off when done
    for pin in pins:
        board.digital[pin].write(0)

    # persist current position to disk
    if count_logical_steps:
        _persist_step_count()

def set_speed(current_delay=None):
    global step_delay
    current_delay = step_delay if current_delay is None else current_delay
    print(f"Current speed: {int(current_delay*1000)} ms per step")
    print("Increasing ms/step results in slower rotation, with 1ms being fastest")
    while True:
        try:
            val = input("Enter new speed in ms per step (e.g., 15), or ENTER to keep: ").strip()
            if val == '':
                print("Keeping current speed.")
                return step_delay
            ms = int(val)
            if ms < MIN_STEP_DELAY_MS or ms > MAX_STEP_DELAY_MS:
                print(f"Enter a value between {MIN_STEP_DELAY_MS} and {MAX_STEP_DELAY_MS} ms.")
                continue
            print(f"Speed set to {ms} ms per step.")
            return _set_step_delay_ms(ms, persist=True)
        except ValueError:
            print("Invalid input. Enter a number.")

def _sensor_state(pin):
    #Return True if BLOCKED, False if OPEN (None treated as last seen).
    val = board.digital[pin].read()
    return bool(val) if val is not None else True  # default to BLOCKED if None

def _seek_open_edge(pin, approach_dir, step_delay=0.004):
    
    #move until we're exactly on the BLOCKED->OPEN edge, approaching in the given direction (for repeatable indexing).

    # If currently OPEN, step until BLOCKED
    while _sensor_state(pin) is False:
        move_stepper(seq, 1, step_delay, approach_dir)

    # Now we're BLOCKED: step until it flips to OPEN (that's the edge)
    while _sensor_state(pin) is True:
        move_stepper(seq, 1, step_delay, approach_dir)
    # Now we're at the edge (first OPEN after BLOCKED), aligned reproducibly.

def _measure_one_cycle_steps(pin, direction, step_delay=0.003, max_steps=100000):
    #Starting *exactly at the OPEN edge*, count steps for one full sensor cycle: 
    # OPEN -> (leave OPEN to BLOCKED) -> return to OPEN. Returns step count.
    
    steps = 0
    left_open = False
    prev = False  # we start on OPEN edge by contract
    for _ in range(max_steps):
        move_stepper(seq, 1, step_delay, direction)
        steps += 1
        cur = _sensor_state(pin)
        if not left_open:
            # wait until we leave OPEN (hit BLOCKED)
            if cur is True:
                left_open = True
        else:
            # after we've been BLOCKED, look for OPEN again
            if cur is False:
                return steps
        prev = cur
    # If we somehow didn't complete a cycle, return what we counted.
    return steps

def _sensor_label(pin):
    if pin == optical_pin2:
        return "S2 (worm gear)"
    if pin == optical_pin:
        return "S1 (motor disk)"
    return f"pin {pin}"

def calibrate_directional_bias(pin=optical_pin2, approach_dir=1, verbose=True):
    """
    Measures:
      - FORWARD steady steps for 1 sensor cycle
      - REVERSE steps for first cycle (includes backlash from direction change)
      - REVERSE steps for second cycle (steady reverse, no direction change)
    Computes:
      BACKLASH_STEPS = first_reverse - reverse_steady
      DIR_REVERSE_SCALE = forward_steady / reverse_steady
    """
    global DIR_REVERSE_SCALE, BACKLASH_STEPS
    label = _sensor_label(pin)
    _log_calibration_event(f"Directional bias calibration started on {label} with approach_dir={approach_dir}")

    # Index to a consistent edge in forward direction
    _seek_open_edge(pin, approach_dir=approach_dir, step_delay=0.004)

    # 1) Forward steady cycle
    f_steps = _measure_one_cycle_steps(pin, direction=+1, step_delay=0.003)

    # We should now be again on the OPEN edge, but 1 rev forward.
    # 2) First reverse cycle (this includes backlash due to direction change)
    r_first = _measure_one_cycle_steps(pin, direction=-1, step_delay=0.003)

    # Now we are back at an OPEN edge again.
    # 3) Second reverse cycle (steady reverse, no direction change)
    r_steady = _measure_one_cycle_steps(pin, direction=-1, step_delay=0.003)

    # Compute compensation
    backlash = max(0, r_first - r_steady)  # backlash can't be negative
    reverse_scale = (f_steps / r_steady) if r_steady > 0 else 1.0

    # Save results
    BACKLASH_STEPS = int(round(backlash))
    DIR_REVERSE_SCALE = float(reverse_scale)

    if verbose:
        print(f"=== Directional Bias Calibration: {label} ===")
        print(f"Forward steady rev steps:        {f_steps}")
        print(f"Reverse first rev (with backlash): {r_first}")
        print(f"Reverse steady rev steps:         {r_steady}")
        print(f"-> BACKLASH_STEPS:               {BACKLASH_STEPS}")
        print(f"-> DIR_REVERSE_SCALE:            {DIR_REVERSE_SCALE:.6f}")
        if abs(DIR_REVERSE_SCALE - 1.0) < 0.01:
            print("Note: Reverse scale ~1.0 (little steady-state directional bias).")
        else:
            print("Note: Non-unity reverse scale detected (steady-state bias).")

    # if _cal is not None:
    #     _cal.BACKLASH_STEPS = int(BACKLASH_STEPS)
    #     _cal.DIR_REVERSE_SCALE = float(DIR_REVERSE_SCALE)
    #     save_cal(_cal)

    out = {
        "forward_steps": f_steps,
        "reverse_first": r_first,
        "reverse_steady": r_steady,
        "backlash_steps": BACKLASH_STEPS,
        "reverse_scale": DIR_REVERSE_SCALE,
    }
    # SAVE:
    path = save_cal(
        STATE_DIR,
        backlash_steps=BACKLASH_STEPS,
        reverse_scale=DIR_REVERSE_SCALE
    )
    print(f"[Cal] Backlash saved to: {path}")
    _log_calibration_event(
        f"{label}: forward={f_steps}, reverse_first={r_first}, reverse_steady={r_steady}, "
        f"backlash={BACKLASH_STEPS}, reverse_scale={DIR_REVERSE_SCALE:.6f}"
    )
    return out

def calibration():
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home, steps_per_rev, revs_per_rotation, steps_since_rev
    print("\n--- Calibration ---")
    print("Press 1 to calibrate the SMALL disk (optical switch 1).")
    print("Press 2 to calibrate the LARGE worm gear disk (optical switch 2).")
    print("Press 3 to calibrate directional bias/backlash on LARGE worm gear disk (optical switch 2).")
    print("Press 4 to calibrate directional bias/backlash on SMALL disk (optical switch 1).")
    print("Press Q to quit back to main menu.")

    while True:
        choice = input("Select calibration: 1 (small disk), 2 (large disk), 3 (S2 backlash), 4 (S1 backlash), Q (quit): ").strip().lower()
        if choice == '1':
            # --- Steps per rev FORWARD ---
            print("Calibrating steps per revolution (SMALL disk, FORWARD)...")
            _log_calibration_event("S1 steps/rev calibration started")
            transition_sequence = ["OPEN"]

            time.sleep(3)
            # Move to slit OPEN if not already
            print("Ensuring small disk slit is OPEN...")
            while True:
                monitor_sensors()
                if not board.digital[optical_pin].read():
                    step_count = 0
                    print("Switch 1 open, small disk slit open")
                    #last_switch1_state = "OPEN"
                    break
                move_stepper(seq, 1, step_delay=0.015, direction=1)

            time.sleep(2)

            initial_step_count = step_count
            initial_rev_count = rev_count
            # Now do one full revolution (open->block->open)
            print("Rotating one full revolution of small disk...")
            initial_state = board.digital[optical_pin].read()
            found_blocked = False
            while True:
                monitor_sensors()
                move_stepper(seq, 1, step_delay=0.015, direction=1)
                state = board.digital[optical_pin].read()
                if not found_blocked and state:
                    found_blocked = True
                if found_blocked and not state:
                    print("Completed one revolution (open->blocked->open).")
                    break
            print("Total steps for one revolution of small disk:", step_count)
            steps_per_rev = step_count-initial_step_count

            path = save_cal(STATE_DIR, s1_steps_per_rev=steps_per_rev)

            print(f"[Cal] S1 steps/rev saved to: {path}")
            _log_calibration_event(f"S1 steps/rev calibration saved: steps_per_rev={steps_per_rev}")
        
            break

        elif choice == '2':
            print("Calibrating worm gear...")
            _log_calibration_event("S2 steps/rev calibration started")
            # Move to home position first
            print("Homing worm gear (large disk) before calibration...please wait...")
            time.sleep(4)
            if not go_home2():
                print("[Cal S2] Could not establish Disk Home before calibration.")
                _log_calibration_event("S2 steps/rev calibration aborted because homing failed")
                return
            print("Big gear homed, begining calibration")

            last_switch2_state = "OPEN"
            found_blocked = False
            
            dirch = input("Approach S2 edge in direction [+/-] (ENTER=+): ").strip()
            approach_dir = -1 if dirch == "-" else +1
            steps = calibrate_s2_steps_per_rev(approach_dir=approach_dir)

            #_run_cancellable(calibrate_s2_steps_per_rev, approach_dir=approach_dir)  # optional

            if steps is None:
                print("[Cal S2] Failed to measure a full cycle. Check sensor and try again.")
                _log_calibration_event("S2 steps/rev calibration failed to measure a full cycle")
                return
            # (S2 calibration is already persisted by calibrate_s2_steps_per_rev)
            _log_calibration_event(f"S2 steps/rev calibration saved: steps_per_rev={steps}")
            if not go_home2():
                print("[Cal S2] Calibration finished, but re-home failed afterward.")
                _log_calibration_event("S2 calibration post-home failed")

        elif choice == '3':
            print("Calibrating directional bias/backlash on large worm gear disk (sensor 2)...")
            result = calibrate_directional_bias(pin=optical_pin2, approach_dir=+1, verbose=True)

        elif choice == '4':
            print("Calibrating directional bias/backlash on small disk (sensor 1)...")
            result = calibrate_directional_bias(pin=optical_pin, approach_dir=+1, verbose=True)


        elif choice.lower() == 'q':
            print("Exiting calibration menu.\n")
            return

        else:
            print("Invalid input. Enter 1, 2, 3, 4, or Q.")

    return

def _ensure_cal_ready():
    """Make sure _cal is a dict with wl_refs/wl_model present."""
    global _cal
    if not isinstance(_cal, dict):
        _cal = {}
    _cal.setdefault("wl_refs", [])
    _cal.setdefault("wl_model", None)
    return _cal

def _wl_path(state_dir):
    return os.path.join(state_dir, "wavelength_cal.json")

def _wl_load(state_dir):
    # prefer your provided cal_store API if available
    try:
        return load_wl_cal(state_dir)
    except Exception:
        pass
    # fallback: plain file
    p = _wl_path(state_dir)
    if os.path.exists(p):
        with open(p, "r") as f:
            return json.load(f)
    return {"refs": [], "model": None}

def _wl_save(state_dir, refs, model):
    # prefer your provided cal_store API if available
    try:
        save_wl_cal(state_dir, refs=refs, model=model)
        return
    except Exception:
        pass
    # fallback: plain file
    p = _wl_path(state_dir)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w") as f:
        json.dump({"refs": refs, "model": model}, f, indent=2, sort_keys=True)

def restore_wl_from_backup():
    """
    Restore wavelength refs/model from state/wavelength_cal_backup.json.
    Updates _cal and WL (if present) and persists via _wl_save(refs, model).
    Falls back to writing state/wavelength_cal.json directly if _wl_save fails.
    """
    global _cal, WL
    _ensure_cal_ready()

    backup_path = os.path.join(STATE_DIR, "wavelength_cal_backup.json")
    target_path = os.path.join(STATE_DIR, "wavelength_cal.json")

    try:
        with open(backup_path, "r") as f:
            data = json.load(f)

        refs = data.get("refs", [])
        model = data.get("model")

        if not isinstance(refs, list):
            raise ValueError("backup 'refs' must be a list")
        if model is not None and not isinstance(model, dict):
            raise ValueError("backup 'model' must be an object or null")

        # Update in-memory store
        _cal["wl_refs"] = refs
        _cal["wl_model"] = model

        # Mirror legacy WL dict if present
        try:
            if isinstance(WL, dict):
                WL["refs"] = list(refs)
                WL["model"] = dict(model) if isinstance(model, dict) else None
        except Exception:
            pass

        # Persist using your helper (positional args!)
        try:
            _wl_save(STATE_DIR, refs, model)           # <-- IMPORTANT: positional, not kwargs
        except Exception:
             # Hard fallback: write the JSON directly
                os.makedirs(STATE_DIR, exist_ok=True)
                with open(target_path, "w") as out:
                    json.dump({
                        "version": data.get("version", 1),
                        "saved_at": time.time(),
                        "refs": refs,
                        "model": model
                    }, out, indent=2, sort_keys=True)

        print(f"[WL Cal] Restored {len(refs)} refs; model: {_wl_model_summary(model)} from {backup_path}")
        _log_calibration_event(f"Wavelength backup restored: refs={len(refs)}, model={_wl_model_summary(model)}")

    except FileNotFoundError:
        print(f"[WL Cal] Backup not found: {backup_path}")
    except Exception as e:
        print(f"[WL Cal] Restore failed: {e}")


def wl_cal_menu():
    while True:
        # Always load fresh from disk to avoid stale in-memory state
        WL = _wl_load(STATE_DIR)
        refs = WL.get("refs", [])
        model = WL.get("model")

        print("\n--- Wavelength Calibration ---")
        print("1) Add reference using CURRENT angle θ")
        print("2) Add reference by ENTERING angle θ")
        print("3) Fit and save affine Littrow model")
        print("3z) Fit and save affine Littrow model with a=0")
        print("3s) Fit and save A·sin(θ + θ0) model")
        print("3c) Fit and save A·sinθ + B·cosθ model")
        print("4) Show current references and model")
        print("5) Use Backup model")
        print("6) ZWO-assisted line scan -> add reference")
        print("J) Jog Mode")
        print("x) Clear ALL wavelength refs and model (reset)")
        print("q) Back")
        ch = input("Select: ").strip().lower()

        if ch == "1":
            try:
                lam = float(input("λ (nm): ").strip())
                m   = int(input("order m (integer, nonzero): ").strip())
            except Exception:
                print("Invalid λ or m.")
                continue

            theta_now = _steps_to_theta_deg(step_count)
            refs.append({"theta_deg": float(theta_now), "lambda_nm": lam, "order_m": int(m)})
            _wl_save(STATE_DIR, refs, model)
            print(f"[WL Cal] Added ref: θ={theta_now:.6f}°, λ={lam} nm, m={m}")
            _log_calibration_event(
                f"Wavelength ref added from current angle: theta={theta_now:.6f} deg, lambda={lam} nm, order={m}"
            )

        elif ch == "2":
            try:
                theta = float(input("θ (deg): ").strip())
                lam   = float(input("λ (nm): ").strip())
                m     = int(input("order m (integer, nonzero): ").strip())
            except Exception:
                print("Invalid entry.")
                continue
            refs.append({"theta_deg": float(theta), "lambda_nm": lam, "order_m": int(m)})
            _wl_save(STATE_DIR, refs, model)
            print(f"[WL Cal] Added ref: θ={theta:.6f}°, λ={lam} nm, m={m}")
            _log_calibration_event(
                f"Wavelength ref added by entry: theta={theta:.6f} deg, lambda={lam} nm, order={m}"
            )

        elif ch in ("3", "3z", "3s", "3c"):
            try:
                if ch == "3z":
                    a, b = _fit_littrow_through_origin(refs)
                    model = {"kind": "affine_littrow", "a": float(a), "b": float(b)}
                elif ch == "3":
                    a, b = _fit_littrow_linear(refs)
                    model = {"kind": "affine_littrow", "a": float(a), "b": float(b)}
                elif ch == "3s":
                    sin_coeff, cos_coeff = _fit_sin_cos_model(refs)
                    amplitude_nm = math.hypot(sin_coeff, cos_coeff)
                    theta0_deg = math.degrees(math.atan2(cos_coeff, sin_coeff)) if amplitude_nm else 0.0
                    model = {
                        "kind": "sin_theta_offset",
                        "amplitude_nm": float(amplitude_nm),
                        "theta0_deg": float(theta0_deg),
                    }
                else:
                    sin_coeff, cos_coeff = _fit_sin_cos_model(refs)
                    model = {
                        "kind": "sin_cos",
                        "sin_coeff_nm": float(sin_coeff),
                        "cos_coeff_nm": float(cos_coeff),
                    }

                model = _normalize_wl_model(model)
                _wl_save(STATE_DIR, refs, model)
                print(f"[WL Cal] Fit saved. {_wl_model_summary(model)}")
                _log_calibration_event(f"Wavelength model fit saved: {_wl_model_summary(model)}")

                # --- Fit quality report ---
                try:
                    res = _wl_model_residuals(refs, model)
                    if res:
                        rms = (sum(r*r for r in res) / len(res)) ** 0.5
                        mx = max(abs(r) for r in res)
                        print(f"[WL Cal] Residuals on λ/m: RMS={rms:.6f} nm, max={mx:.6f} nm  (computed on your refs)")
                except Exception:
                    pass


                # Also scrub any legacy in-memory copies so they don't “come back”
                try:
                    if isinstance(_cal, dict):
                        _cal.pop("wl_refs", None)
                        _cal.pop("wl_model", None)
                except Exception:
                    pass

            except Exception as e:
                print(f"[WL Cal] Fit failed: {e}")

        elif ch == "4":
            print("Refs:")
            if not refs:
                print("  (none)")
            else:
                for i, r in enumerate(refs):
                    print(f"  {i+1}: θ={r['theta_deg']:.6f}°, λ={r['lambda_nm']} nm, m={r['order_m']}")
            if model is not None:
                print(f"Model: {_wl_model_summary(model)}")
                res = _wl_model_residuals(refs, model)
                if res:
                    rms = (sum(r*r for r in res) / len(res)) ** 0.5
                    mx = max(abs(r) for r in res)
                    print(f"Residuals on λ/m: RMS={rms:.6f} nm, max={mx:.6f} nm")
            else:
                print("Model: (none) — ideal Littrow fallback (affine Littrow, a=0, b=1)")

        elif ch == "5":
            # Load backup model from cal_store if available
            try:
                restore_wl_from_backup()
                _log_calibration_event("Wavelength model restored from backup")
            except Exception as e:
                print(f"[WL Cal] Could not load backup model: {e}")

        elif ch == "6":
            print("Launching ZWO-assisted line scan... (press 'q' or Esc to cancel)")
            _log_calibration_event("Starting ZWO-assisted wavelength reference capture")
            _run_cancellable(zwo_assisted_reference_capture)

        elif ch == "j":
            drain_stdin()
            jog_mode2()
            drain_stdin()

        elif ch == "x":
            confirm = input("Clear ALL wavelength refs and model? (y/q): ").strip().lower()
            if confirm != "y":
                print("Clear cancelled.")
                continue

            # Reset WL file
            refs = []
            model = None
            _wl_save(STATE_DIR, refs, model)
            print("[WL Cal] Cleared refs and model (wavelength_cal.json).")
            _log_calibration_event("Cleared all wavelength references and model")

            # Also nuke legacy fields in _cal so they can't re-populate the UI
            try:
                if isinstance(_cal, dict):
                    _cal.pop("wl_refs", None)
                    _cal.pop("wl_model", None)
            except Exception:
                pass

        elif ch == "q":
            return
        else:
            print("Invalid option.")


def _parse_roi(roi_text):
    parts = [p.strip() for p in roi_text.split(",") if p.strip()]
    if len(parts) != 4:
        raise ValueError("ROI must be x,y,w,h")
    x, y, w, h = (int(p) for p in parts)
    if x < 0 or y < 0 or w <= 0 or h <= 0:
        raise ValueError("ROI values must be non-negative and width/height must be > 0")
    return x, y, w, h

def _quadratic_peak_center(xs, ys):
    if len(xs) != len(ys) or not xs:
        raise ValueError("Need matching, non-empty x/y samples.")
    idx = max(range(len(ys)), key=lambda i: ys[i])
    x_peak = float(xs[idx])
    if 0 < idx < len(ys) - 1:
        y1, y2, y3 = ys[idx - 1], ys[idx], ys[idx + 1]
        denom = y1 - 2.0 * y2 + y3
        if abs(denom) > 1e-12:
            dx = float(xs[idx] - xs[idx - 1])
            offset = 0.5 * (y1 - y3) / denom * dx
            if abs(offset) <= abs(dx):
                x_peak = float(xs[idx]) + offset
    return x_peak, idx

def _load_numpy():
    try:
        import numpy as np
        return np
    except Exception as e:
        raise RuntimeError("NumPy is required for ZWO-assisted capture. Install it with: pip install numpy") from e

def _load_zwoasi(lib_path=None):
    try:
        import zwoasi as asi
    except Exception as e:
        raise RuntimeError(
            "Python package 'zwoasi' is not installed. Install it and set ZWO_ASI_LIB to your ASICamera2 SDK library."
        ) from e

    if not getattr(_load_zwoasi, "_initialized", False):
        sdk_path = lib_path or os.getenv("ZWO_ASI_LIB")
        try:
            if sdk_path:
                asi.init(sdk_path)
            else:
                asi.init()
        except TypeError:
            if not sdk_path:
                raise RuntimeError("Set ZWO_ASI_LIB to the ASICamera2 SDK library path before using the ZWO routine.")
            asi.init(sdk_path)
        except Exception as e:
            if sdk_path:
                raise RuntimeError(f"Could not initialize ASICamera2 SDK from '{sdk_path}': {e}") from e
            raise RuntimeError("Could not initialize the ZWO ASI SDK. Set ZWO_ASI_LIB to the ASICamera2 SDK library path.") from e
        _load_zwoasi._initialized = True

    return asi

def _zwo_open_camera(camera_index=0, exposure_ms=10.0, gain=0, lib_path=None):
    asi = _load_zwoasi(lib_path=lib_path)
    num_cameras = asi.get_num_cameras()
    if num_cameras < 1:
        raise RuntimeError("No ZWO cameras detected.")
    if camera_index < 0 or camera_index >= num_cameras:
        raise RuntimeError(f"Camera index {camera_index} is out of range (found {num_cameras}).")

    camera = asi.Camera(camera_index)
    try:
        camera.stop_video_capture()
    except Exception:
        pass
    try:
        camera.stop_exposure()
    except Exception:
        pass

    try:
        camera.set_image_type(asi.ASI_IMG_RAW8)
    except Exception:
        pass
    try:
        camera.set_control_value(asi.ASI_EXPOSURE, int(round(exposure_ms * 1000.0)))
    except Exception:
        pass
    try:
        camera.set_control_value(asi.ASI_GAIN, int(gain))
    except Exception:
        pass

    return asi, camera

def _zwo_frame_to_array(frame, camera, np):
    if hasattr(frame, "shape"):
        return frame
    if isinstance(frame, memoryview):
        frame = frame.tobytes()
    if isinstance(frame, (bytes, bytearray)):
        width, height, _, _ = camera.get_roi_format()
        arr = np.frombuffer(frame, dtype=np.uint8)
        expected = int(width) * int(height)
        if arr.size < expected:
            raise RuntimeError(f"Captured frame is too small ({arr.size} bytes, expected {expected}).")
        return arr[:expected].reshape((int(height), int(width)))
    raise RuntimeError(f"Unsupported frame type from ZWO camera: {type(frame)!r}")

def _zwo_capture_metric(camera, roi=None, average_frames=3):
    np = _load_numpy()
    metrics = []
    for _ in range(max(1, int(average_frames))):
        frame = None
        if hasattr(camera, "capture_video_frame"):
            try:
                frame = camera.capture_video_frame()
            except Exception:
                frame = None
        if frame is None and hasattr(camera, "capture"):
            frame = camera.capture()
        if frame is None:
            raise RuntimeError("Could not capture a frame from the ZWO camera.")

        arr = _zwo_frame_to_array(frame, camera, np)
        if roi:
            x, y, w, h = roi
            if x >= arr.shape[1] or y >= arr.shape[0]:
                raise RuntimeError(f"ROI {roi} is outside the camera frame {arr.shape[1]}x{arr.shape[0]}.")
            arr = arr[y:min(y + h, arr.shape[0]), x:min(x + w, arr.shape[1])]
        region = arr.astype(np.float32, copy=False)
        background = float(np.percentile(region, 10.0))
        metrics.append(float(np.clip(region - background, 0.0, None).sum()))
    return sum(metrics) / len(metrics)

def zwo_assisted_reference_capture():
    try:
        lam = float(_prompt_or_quit("Known line wavelength λ (nm) (q=quit): "))
        order_m = int(_prompt_or_quit("Diffraction order m (int, nonzero) (q=quit): "))
        if order_m == 0:
            raise ValueError("order m must be nonzero")

        premove = input("Pre-move to the current model prediction first? [y/N]: ").strip().lower() == "y"
        mode = None
        if premove:
            mode = _ask_approach_mode()

        half_width_steps = int(_prompt_or_quit("Scan half-width in steps (q=quit): "))
        step_size_steps = int(_prompt_or_quit("Scan step size in steps (q=quit): "))
        if half_width_steps <= 0 or step_size_steps <= 0:
            raise ValueError("Scan width and step size must be positive integers.")
        if step_size_steps > half_width_steps:
            raise ValueError("Scan step size must be <= scan half-width.")

        settle_raw = input("Settle time per point in seconds (ENTER=0.05): ").strip()
        settle_s = float(settle_raw) if settle_raw else 0.05

        avg_raw = input("Frames to average per point (ENTER=3): ").strip()
        average_frames = int(avg_raw) if avg_raw else 3

        camera_idx_raw = input("ZWO camera index (ENTER=0): ").strip()
        camera_index = int(camera_idx_raw) if camera_idx_raw else 0

        exposure_raw = input("Exposure in ms (ENTER=10): ").strip()
        exposure_ms = float(exposure_raw) if exposure_raw else 10.0

        gain_raw = input("Gain (ENTER=0): ").strip()
        gain = int(gain_raw) if gain_raw else 0

        lib_path_raw = input("ASICamera2 SDK library path (ENTER=use $ZWO_ASI_LIB): ").strip()
        lib_path = lib_path_raw or None

        roi_raw = input("ROI x,y,w,h (ENTER=full frame): ").strip()
        roi = _parse_roi(roi_raw) if roi_raw else None
    except _QuitToMain:
        raise
    except Exception as e:
        print(f"[ZWO] Input error: {e}")
        return

    if premove:
        print(f"[ZWO] Moving near λ={lam} nm, m={order_m} using the current wavelength model...")
        move_to_wavelength_nonlinear(lam, order_m, mode=mode)

    center_steps = int(step_count)
    start_steps = center_steps - half_width_steps
    total_points = (2 * half_width_steps) // step_size_steps + 1
    sample_steps = []
    sample_metrics = []
    camera = None
    _log_calibration_event(
        f"ZWO scan start: lambda={lam} nm, order={order_m}, center_step={center_steps}, "
        f"half_width={half_width_steps}, step_size={step_size_steps}, roi={roi}"
    )

    try:
        _, camera = _zwo_open_camera(
            camera_index=camera_index,
            exposure_ms=exposure_ms,
            gain=gain,
            lib_path=lib_path,
        )
        if hasattr(camera, "start_video_capture"):
            camera.start_video_capture()

        print(
            f"[ZWO] Scanning {total_points} points over ±{half_width_steps} steps "
            f"around step {center_steps} for λ={lam} nm, m={order_m}."
        )

        _move_to_target_steps_fixed_approach(start_steps, final_dir=+1)

        for i in range(total_points):
            _check_cancel()
            _sleep_with_cancel(settle_s)
            metric = _zwo_capture_metric(camera, roi=roi, average_frames=average_frames)
            sample_steps.append(int(step_count))
            sample_metrics.append(metric)
            print(
                f"[ZWO] step={step_count:6d} θ={_steps_to_theta_deg(step_count):9.6f}° "
                f"metric={metric:12.3f}"
            )
            if i != total_points - 1:
                move_biased(step_size_steps, EDGE_STEP_DELAY, +1)

    except _QuitToMain:
        raise
    except Exception as e:
        print(f"[ZWO] Scan failed: {e}")
        return
    finally:
        if camera is not None:
            try:
                camera.stop_video_capture()
            except Exception:
                pass
            try:
                camera.close()
            except Exception:
                pass

    peak_step, peak_idx = _quadratic_peak_center(sample_steps, sample_metrics)
    peak_theta = _steps_to_theta_deg(peak_step)
    peak_metric = sample_metrics[peak_idx]
    refs = list(_wl_load(STATE_DIR).get("refs", []))
    model = _wl_load(STATE_DIR).get("model")
    refs.append({"theta_deg": float(peak_theta), "lambda_nm": float(lam), "order_m": int(order_m)})
    _wl_save(STATE_DIR, refs, model)

    print(
        f"[ZWO] Added ref from peak fit: θ={peak_theta:.6f}°, λ={lam} nm, m={order_m} "
        f"(peak metric≈{peak_metric:.3f})."
    )
    _log_calibration_event(
        f"ZWO ref added: theta={peak_theta:.6f} deg, lambda={lam} nm, order={order_m}, peak_metric={peak_metric:.3f}"
    )
    if 0 < peak_idx < len(sample_steps) - 1:
        print(f"[ZWO] Quadratic peak center at step {peak_step:.3f} between sampled points.")
    else:
        print("[ZWO] Peak landed on a scan edge; widen the scan window if you want a cleaner fit.")

    move_to_peak = input("Move to the fitted peak center now? [Y/n]: ").strip().lower()
    if move_to_peak not in ("n", "no"):
        _move_to_target_steps_fixed_approach(int(round(peak_step)), final_dir=+1)

    fit_now = input(
        "Immediate refit? [a]=affine, [o]=origin, [s]=sin(θ+θ0), [c]=sinθ+Bcosθ, ENTER=skip: "
    ).strip().lower()
    try:
        if fit_now == "a":
            a, b = _fit_littrow_linear(refs)
            model = {"kind": "affine_littrow", "a": float(a), "b": float(b)}
        elif fit_now == "o":
            a, b = _fit_littrow_through_origin(refs)
            model = {"kind": "affine_littrow", "a": float(a), "b": float(b)}
        elif fit_now == "s":
            sin_coeff, cos_coeff = _fit_sin_cos_model(refs)
            amplitude_nm = math.hypot(sin_coeff, cos_coeff)
            theta0_deg = math.degrees(math.atan2(cos_coeff, sin_coeff)) if amplitude_nm else 0.0
            model = {
                "kind": "sin_theta_offset",
                "amplitude_nm": float(amplitude_nm),
                "theta0_deg": float(theta0_deg),
            }
        elif fit_now == "c":
            sin_coeff, cos_coeff = _fit_sin_cos_model(refs)
            model = {
                "kind": "sin_cos",
                "sin_coeff_nm": float(sin_coeff),
                "cos_coeff_nm": float(cos_coeff),
            }
        else:
            model = None

        if model is not None:
            model = _normalize_wl_model(model)
            _wl_save(STATE_DIR, refs, model)
            res = _wl_model_residuals(refs, model)
            rms = (sum(r*r for r in res) / len(res)) ** 0.5 if res else 0.0
            mx = max(abs(r) for r in res) if res else 0.0
            print(f"[ZWO] Model saved. {_wl_model_summary(model)}")
            print(f"[ZWO] Residuals on λ/m: RMS={rms:.6f} nm, max={mx:.6f} nm")
            _log_calibration_event(
                f"ZWO refit saved: {_wl_model_summary(model)}; residual_rms={rms:.6f} nm, residual_max={mx:.6f} nm"
            )
    except Exception as e:
        print(f"[ZWO] Fit failed: {e}")


def _log_homing_event(message):
    logging.info(f"[Home] {message}")

def _log_calibration_event(message):
    logging.info(f"[Cal] {message}")

def _clear_sensor_terminal_line():
    global _last_sensor_terminal_width, _last_sensor_terminal_time
    if _last_sensor_terminal_width <= 0:
        _last_sensor_terminal_time = 0.0
        return
    sys.stdout.write("\r" + (" " * _last_sensor_terminal_width) + "\r")
    sys.stdout.flush()
    _last_sensor_terminal_width = 0
    _last_sensor_terminal_time = 0.0

def _print_sensor_snapshot(message, *, inline=True):
    global _last_sensor_terminal_width
    if inline:
        padded = message
        if len(message) < _last_sensor_terminal_width:
            padded = message + (" " * (_last_sensor_terminal_width - len(message)))
        sys.stdout.write("\r" + padded)
        sys.stdout.flush()
        _last_sensor_terminal_width = len(padded)
        return
    _clear_sensor_terminal_line()
    print(message)

def _set_debug_mode(enabled):
    global DEBUG_MOTION_LOGGING, DEBUG_TERMINAL_OUTPUT
    DEBUG_MOTION_LOGGING = bool(enabled)
    DEBUG_TERMINAL_OUTPUT = bool(enabled)

def toggle_debug_mode():
    _set_debug_mode(not DEBUG_MOTION_LOGGING)
    state = "ON" if DEBUG_MOTION_LOGGING else "OFF"
    print(f"[Debug] Continuous sensor debug is {state}.")
    if DEBUG_MOTION_LOGGING:
        print("[Debug] monitor_sensors() will stream every sample to the log and terminal.")
    else:
        print("[Debug] monitor_sensors() will log sensor changes only, except during jog mode.")

def _sensor_state_label(value):
    return (
        "Waiting" if value is None else
        "BLOCKED" if value else
        "OPEN"
    )

def _direction_label(direction):
    if direction > 0:
        return "CW"
    if direction < 0:
        return "CCW"
    return "IDLE"

def _motor_direction(stage_direction):
    return (1 if stage_direction > 0 else -1) * MOTOR_SEQUENCE_SIGN

def _seq_index_from_step_count(step_value):
    if not seq:
        return 0
    return (int(step_value) * MOTOR_SEQUENCE_SIGN) % len(seq)

def _format_sensor_snapshot(state1, state2, *, reason=None, debug=False):
    if reason == "jog":
        theta_text = "--"
        try:
            theta_text = f"{_steps_to_theta_deg(step_count):+9.4f}deg"
        except Exception:
            pass
        ref = "BOTH-OPEN" if (state1 == "OPEN" and state2 == "OPEN") else ("S2-OPEN" if state2 == "OPEN" else "--")
        return (
            f"JOG | dir={_direction_label(JOG_DISPLAY_DIR):<4} | spd={_delay_to_ms(step_delay):4d}ms | step={step_count:+8d} | "
            f"theta={theta_text} | S1={state1:<7} | S2={state2:<7} | ref={ref}"
        )
    tag = "[Sensors:debug]" if debug else "[Sensors]"
    msg = f"{tag} S1={state1} | S2={state2} | step={step_count}"
    if reason:
        msg += f" | {reason}"
    return msg

def _log_sensor_snapshot(state1, state2, *, reason=None, debug=False):
    msg = _format_sensor_snapshot(state1, state2, reason=reason, debug=debug)
    logging.info(msg)
    return msg

def monitor_sensors(debug=False, reason=None):
    global step_count, rev_count, switch1, switch2, last_states, home, _last_sensor_terminal_time

    switch1 = board.digital[optical_pin].read()
    switch2 = board.digital[optical_pin2].read()

    state1 = _sensor_state_label(switch1)
    state2 = _sensor_state_label(switch2)

    changed = (state1 != last_states[0]) or (state2 != last_states[1])
    log_every = debug or DEBUG_MOTION_LOGGING
    print_every = debug or DEBUG_TERMINAL_OUTPUT or JOG_STREAM_SENSORS

    if log_every:
        _log_sensor_snapshot(state1, state2, reason=reason, debug=True)
    elif changed:
        _log_sensor_snapshot(state1, state2, reason=reason, debug=False)

    if print_every:
        now = time.time()
        should_refresh = changed or (now - _last_sensor_terminal_time) >= JOG_TERMINAL_REFRESH_S
        if should_refresh:
            msg = _format_sensor_snapshot(state1, state2, reason=reason, debug=(debug or DEBUG_MOTION_LOGGING))
            _print_sensor_snapshot(msg, inline=True)
            _last_sensor_terminal_time = now
    last_states = [state1, state2]

    if state2 == "OPEN":
        #print("Home position detected!.")
        #step_count = 0
        #rev_count = 0
        home = True

def read_state(pin):
    """Return True if BLOCKED, False if OPEN. Treat None as last known; assume BLOCKED at startup."""
    val = board.digital[pin].read()
    return bool(val) if val is not None else True


def _read_blocked(pin, retries=5):
    # True=BLOCKED, False=OPEN, retry to avoid None flicker
    for _ in range(retries):
        v = board.digital[pin].read()
        if v is not None:
            return bool(v)
        time.sleep(0.002)
    return True  # conservative default

def _step_until_state(pin, target_blocked, direction, step_delay, max_steps=MAX_FIND_STEPS):
    # Step in 'direction' until sensor matches target_blocked
    for i in range(max_steps):
        if _read_blocked(pin) == target_blocked:
            return i
        move_biased(1, step_delay, direction)
    raise RuntimeError("step_until_state: exceeded MAX_FIND_STEPS")


def _find_B2O_and_measure_open(pin, direction, first_fast=True):
    """
    Force BLOCKED, then step until first OPEN (BLOCKED->OPEN edge),
    continue stepping while OPEN to measure window width (in steps).
    Returns open_width (integer). Ends at OPEN->BLOCKED far edge.
    """

    # Force BLOCKED, then hit the BLOCKED->OPEN edge, then measure OPEN width.
    _step_until_state(pin, True,  direction, FAST_STEP_DELAY if first_fast else EDGE_STEP_DELAY)
    _step_until_state(pin, False, direction, EDGE_STEP_DELAY)


    # Now inside OPEN region, count width while staying OPEN
    width = 0
    for _ in range(MAX_FIND_STEPS):
        if _read_blocked(pin):      # left OPEN → at far edge
            break
        move_biased(1, EDGE_STEP_DELAY, direction)
        width += 1
    if width == 0:
        raise RuntimeError("Open window width measured as 0")
    return width

def _center_on_open_window(pin, approach_dir, first_fast=True):
    """
    Centers the sensor on the OPEN window using the chosen approach_dir.
    We land at the far (OPEN->BLOCKED) edge and reverse by half width.
    Backlash is handled by move_biased().
    """

    global EDGE_HOLDOFF_STEPS

    w = _find_B2O_and_measure_open(pin, direction=approach_dir, first_fast=first_fast)
    EDGE_HOLDOFF_STEPS = max(3, int(0.10 * w))
    half = max(1, w // 2)
    
    move_biased(half, CENTER_STEP_DELAY, -approach_dir)  # reverse half to center
    return w


def _both_open():
    return (not _read_blocked(optical_pin2)) and (not _read_blocked(optical_pin))

def _micro_tweak_around_center(approach_dir):
    """
    Small bounded search to satisfy 'both sensors OPEN', without runaway.
    Tries +dir for a few steps, then -dir, then returns to nearest found.
    """
    if _both_open():
        return True

    moved_fwd = 0
    for i in range(1, TWEAK_RANGE + 1):
        move_biased(1, TWEAK_STEP_DELAY, approach_dir)
        moved_fwd += 1
        if _both_open():
            return True

    # go back past center the other way
    move_biased(moved_fwd, TWEAK_STEP_DELAY, -approach_dir)
    moved_back = 0
    for i in range(1, TWEAK_RANGE + 1):
        move_biased(1, TWEAK_STEP_DELAY, -approach_dir)
        moved_back += 1
        if _both_open():
            return True

    # Return to center-ish
    move_biased(moved_back, TWEAK_STEP_DELAY, approach_dir)
    return _both_open()


def move_until_state(pin, target_blocked, direction, step_delay, max_steps=200000):
    """
    Step until pin reaches target state. Returns steps moved (>=0).
    Aborts safely at max_steps.
    """
    steps = 0
    for _ in range(max_steps):
        cur = read_state(pin)
        if cur == target_blocked:
            return steps
        move_biased(1, step_delay, direction)
        steps += 1
    return steps  # hit limit (fail-safe)

def capture_rising_open(pin, approach_dir, backoff_steps, fast_delay, slow_delay):
    """
    Hysteresis pass:
      1) Ensure we're on BLOCKED side
      2) Approach rising edge (BLOCKED->OPEN) at fast_delay
      3) Back off a little and re-approach more slowly at slow_delay
    Return total steps moved (not used for logic; just for logs).
    """
    total = 0
    # Ensure BLOCKED (move from OPEN to BLOCKED in opposite direction)
    if read_state(pin) is False:  # OPEN
        total += move_until_state(pin, target_blocked=True, direction=-approach_dir, step_delay=fast_delay)

    # First approach to OPEN (fast)
    total += move_until_state(pin, target_blocked=False, direction=approach_dir, step_delay=fast_delay)

    # Back off a bit into BLOCKED
    move_biased(backoff_steps, fast_delay, -approach_dir); total += backoff_steps
    move_until_state(pin, target_blocked=True, direction=-approach_dir, step_delay=fast_delay)  # ensure BLOCKED

    # Second approach to OPEN (slow)
    total += move_until_state(pin, target_blocked=False, direction=approach_dir, step_delay=slow_delay)

    return total

def find_window_edges(pin, direction, step_delay, max_steps=100000):
    """
    Starting wherever we are, find OPEN window edges for `pin` by:
      - Ensure we are BLOCKED
      - Step until OPEN -> that's rising edge (A)
      - Continue until BLOCKED -> that's falling edge (B)
    Returns (A_to_B_width, steps_A, steps_B) relative to where we call it.
    """
    # Ensure BLOCKED
    if read_state(pin) is False:  # OPEN
        move_until_state(pin, target_blocked=True, direction=-direction, step_delay=step_delay)

    # Rising edge to OPEN
    steps_A = move_until_state(pin, target_blocked=False, direction=direction, step_delay=step_delay)

    # Continue to falling edge back to BLOCKED
    steps_B = steps_A + move_until_state(pin, target_blocked=True, direction=direction, step_delay=step_delay)

    width = steps_B - steps_A
    return width, steps_A, steps_B

def _zero_s1_counters():
    global rev_count, steps_since_rev, steps_till_rev, pre_rev_steps, post_rev_steps
    global is_1rev_completed, transition_sequence, last_switch1_state
    rev_count = 0
    steps_since_rev = 0
    steps_till_rev = 0
    pre_rev_steps = 0
    post_rev_steps = 0
    is_1rev_completed = False
    transition_sequence = ["OPEN"]
    last_switch1_state = None


def go_home2():
    """
    Rehome with awareness of current sensor state.

    If S2 is already OPEN (likely near home):
        1) Bump out of the window by REHOME_BUMP_STEPS in the *opposite* of approach_dir
        2) Ensure we're BLOCKED
        3) Do a normal centering pass in approach_dir (slow)

    If S2 is not OPEN:
        - Optional fast pre-roll TOWARD home using approach_dir
        - Center in approach_dir (fast first pass is OK)
    """
    global step_count, _seq_index, _last_move_dir, _rev_scale_err_accum
    _log_homing_event(f"go_home2 start: step_count={step_count}")

    # Keep the useful behavior you wanted: use the signed position estimate to pick
    # which direction to pre-roll/approach from. Positive means CW of home, so move
    # CCW toward home; negative means CCW of home, so move CW toward home.
    if step_count > 0:
        approach_dir = -1
    elif step_count < 0:
        approach_dir = +1
    else:
        approach_dir = HOME_DEFAULT_APPROACH_DIR

    s2_is_open = not _read_blocked(optical_pin2)
    bump_dir = -approach_dir  # leave the OPEN window opposite to the approach

    print(f"[go_home2] step_count={step_count}, s2_open={s2_is_open}, approach_dir={approach_dir}, bump_dir={bump_dir}")
    _log_homing_event(
        f"go_home2 state: step_count={step_count}, s2_open={s2_is_open}, "
        f"approach_dir={approach_dir}, bump_dir={bump_dir}"
    )

    try:
        if s2_is_open:
            # --- ALREADY IN OPEN: do the simple “100-step then normal homing” ---
            move_biased(REHOME_BUMP_STEPS, EDGE_STEP_DELAY, bump_dir)
            _step_until_state(optical_pin2, True, bump_dir, EDGE_STEP_DELAY)  # ensure BLOCKED

            # Now do a single centering pass in the *approach_dir* (slow)
            open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=False)
            print(f"[Rehome@Open] S2 centered. OPEN≈{open_w} steps (dir {approach_dir}).")
            _log_homing_event(f"S2 centered from open window: width≈{open_w} steps, dir={approach_dir}")

        else:
            # Keep the sign-based pre-roll, but only as a helper. Home is still validated
            # exclusively by the sensors before zeroing step_count.
            steps_far = abs(step_count)
            if steps_far > FAR_STEP_THRESHOLD:
                fast_steps = steps_far - FAR_STEP_THRESHOLD
                move_biased(fast_steps, FAST_STEP_DELAY, approach_dir)

            open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=False)
            print(f"[Rehome] S2 centered. OPEN≈{open_w} steps (dir {approach_dir}).")
            _log_homing_event(f"S2 centered after approach: width≈{open_w} steps, dir={approach_dir}")

    except RuntimeError as e:
        # One safe retry in the opposite direction
        alt_dir = -approach_dir
        print(f"[Rehome] Centering failed in dir {approach_dir}: {e}. Retrying in dir {alt_dir}.")
        _log_homing_event(f"Centering failed in dir {approach_dir}: {e}. Retrying in dir {alt_dir}")
        open_w = _center_on_open_window(optical_pin2, alt_dir, first_fast=False)
        print(f"[Rehome] Retry succeeded. OPEN≈{open_w} steps (dir {alt_dir}).")
        _log_homing_event(f"Retry succeeded: width≈{open_w} steps, dir={alt_dir}")
        approach_dir = alt_dir  # downstream offset/tweak should follow the dir actually used

    # Apply per-direction offset from S2 center to true Disk Home
    off = int(HOME_OFFSET_DIR.get(approach_dir, 0))
    if off:
        move_biased(off, CENTER_STEP_DELAY, approach_dir)

    # Final bounded tweak so both sensors are OPEN
    if _micro_tweak_around_center(approach_dir):
        print("Arrived at Disk Home (both sensors OPEN).")
        _log_homing_event("Arrived at Disk Home with both sensors OPEN")
    else:
        print("At reference, but both sensors not OPEN. Adjust TWEAK_RANGE or HOME_OFFSET_DIR.")
        _log_homing_event("At reference but both sensors were not OPEN after micro-tweak")
        return False

    _zero_s1_counters()  # (doesn't touch step_count)
    step_count = 0
    _seq_index = 0
    _last_move_dir = None
    _rev_scale_err_accum = 0.0
    _persist_step_count()
    _log_homing_event("Disk Home established. step_count reset to 0 and persisted")
    return True


#helper functions for optical home offset to be added to the position menu's wavelength to step calculation
def _target_steps_from_disk_home(theta_deg: float) -> int:
    """
    Convert desired grating angle to steps, referenced to DISK HOME,
    by adding the saved optical-home offset.
    """
    _ensure_steps_per_deg()
    base = angle_to_steps(theta_deg)                 # your existing sign convention
    return int(round(OPTICAL_HOME_OFFSET_STEPS + base))


def go_optical_home():
    """
    Go to Disk Home (repeatable, via sensors), then apply the saved optical-home offset.
    """
    global OPTICAL_HOME_OFFSET_STEPS
    if not go_home2():
        print("[OpticalHome] Could not establish Disk Home.")
        return False
    if OPTICAL_HOME_OFFSET_STEPS != 0:
        _stage_move_signed(OPTICAL_HOME_OFFSET_STEPS)
    print(f"[OpticalHome] At optical home (offset {OPTICAL_HOME_OFFSET_STEPS} steps from disk home).")
    return True

def find_optical_home_offset():
    global OPTICAL_HOME_OFFSET_STEPS  # ok at function scope

    # 1) Start from a clean Disk Home
    if not go_home2():
        print("[Offset] Could not establish Disk Home; offset adjust cancelled.")
        return

    drain_stdin()
    session_offset = 0
    running = True
    coarse_nudge = _legacy_steps(10)
    pending = deque()
    pressed = set()
    status_msg = "[Offset] Ready."
    last_render = 0.0
    status_dirty = True

    print("\n--- Set Optical Home Offset ---")
    print(f"Use ↑/↓ to nudge by 1 half-step (PgUp/PgDn = ±{coarse_nudge}). Press Enter or 's' to save, 'r' to reset, 'q'/Esc to cancel.")
    time.sleep(0.5)

    def _nudge(n):
        nonlocal session_offset, status_msg, status_dirty
        if n == 0:
            return
        direction = 1 if n > 0 else -1
        move_exact(abs(n), TWEAK_STEP_DELAY, direction)
        session_offset += n
        status_msg = (
            f"OFFSET | dir={_direction_label(direction):<4} | session={session_offset:+7d} | "
            f"saved={OPTICAL_HOME_OFFSET_STEPS:+7d} | step_count={step_count:+8d} | "
            "Enter/s=save r=reset q=cancel"
        )
        status_dirty = True

    def _key_id(key):
        if hasattr(key, "char") and key.char:
            return key.char.lower()
        return key

    def _queue_once(key, action):
        nonlocal status_msg, status_dirty
        kid = _key_id(key)
        if kid in pressed:
            return
        pressed.add(kid)
        if action[0] == "nudge" and len(pending) >= OFFSET_ADJUST_MAX_PENDING:
            status_msg = "[Offset] Input throttled. Release keys and step again."
            status_dirty = True
            return
        pending.append(action)

    def on_press(key):
        nonlocal running
        try:
            if key == pynput_keyboard.Key.up:
                _queue_once(key, ("nudge", +1))
            elif key == pynput_keyboard.Key.down:
                _queue_once(key, ("nudge", -1))
            elif key == pynput_keyboard.Key.page_up:
                _queue_once(key, ("nudge", +coarse_nudge))
            elif key == pynput_keyboard.Key.page_down:
                _queue_once(key, ("nudge", -coarse_nudge))
            elif key == pynput_keyboard.Key.enter or (hasattr(key, "char") and key.char and key.char.lower() == 's'):
                _queue_once(key, ("save", None))
            elif key == pynput_keyboard.Key.esc or (hasattr(key, "char") and key.char and key.char.lower() == 'q'):
                _queue_once(key, ("cancel", None))
            elif hasattr(key, "char") and key.char and key.char.lower() == 'r':
                _queue_once(key, ("reset", None))
        except Exception as e:
            print(f"[Listener] {e}")

    def on_release(key):
        try:
            pressed.discard(_key_id(key))
        except Exception:
            pass

    listener = _make_keyboard_listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        while running:
            action_executed = False
            if pending:
                action, value = pending.popleft()
                if action == "nudge":
                    _nudge(value)
                    action_executed = True
                elif action == "reset":
                    if session_offset != 0:
                        _nudge(-session_offset)
                    status_msg = (
                        f"OFFSET | dir=IDLE | session={session_offset:+7d} | "
                        f"saved={OPTICAL_HOME_OFFSET_STEPS:+7d} | step_count={step_count:+8d} | "
                        "Reset to Disk Home"
                    )
                    status_dirty = True
                    action_executed = True
                elif action == "save":
                    OPTICAL_HOME_OFFSET_STEPS = session_offset
                    _persist_optical_offset()
                    _maybe_persist_step_count(force=True)
                    try:
                        with open(OPTICAL_OFFSET_FILE, "r") as f:
                            status_msg = (
                                f"[Saved] Optical-home offset set to {OPTICAL_HOME_OFFSET_STEPS} steps "
                                f"(file now has {f.read().strip()})."
                            )
                    except Exception as e:
                        status_msg = f"[Saved] Offset persisted, but readback failed: {e}"
                    running = False
                    break
                elif action == "cancel":
                    status_msg = "[Cancel] Leaving offset unchanged."
                    _maybe_persist_step_count(force=True)
                    running = False
                    break

            monitor_sensors(reason="offset_adjust")
            now = time.time()
            if action_executed or status_dirty or (now - last_render) >= OFFSET_ADJUST_REFRESH_S:
                _print_sensor_snapshot(status_msg, inline=True)
                last_render = now
                status_dirty = False
            time.sleep(0.01)
    finally:
        listener.stop()
        listener.join(timeout=0.2)
        settle_terminal_input()
        _clear_sensor_terminal_line()
        print(status_msg)
    print("Done setting optical-home offset.")



def go_home():
    global step_count, rev_count, steps_till_rev, steps_since_rev, transition_sequence, step_delay
    global home_revs, home_steps, home_pre_rev_steps, home_delta_steps
    print("\nMoving to Home Position...")
    _log_homing_event(
        f"go_home start: rev_count={rev_count}, step_count={step_count}, "
        f"home_revs={home_revs}, home_steps={home_steps}"
    )

    # Calculate total steps needed to get back to home
    # Determine direction: -1 = reverse, 1 = forward
    rev_diff = rev_count 
    step_diff = step_count

    #total steps taking during go_home process
    homing_steps = 0

    #print(f"Current revs: {rev_count}, home revs: {home_revs}")
    #print(f"Current steps: {step_count}, home steps: {home_steps}")
    #print(f"Revolution difference: {rev_diff}, Step difference: {step_diff}")
    _log_homing_event(f"Current revs: {rev_count}, home revs: {home_revs}")
    _log_homing_event(f"Current steps: {step_count}, home steps: {home_steps}")
    _log_homing_event(f"Revolution difference: {rev_diff}, Step difference: {step_diff}")
    # Calculate steps to move before reaching home

    direction = -1 if rev_diff > 0 or (rev_diff == 0 and step_diff > 0) else 1

    #move the post-revs steps 
    #move_stepper(seq, steps_since_rev, step_delay, direction)
    move_biased(steps = 1, step_delay=step_delay, direction=direction)

    homing_steps = homing_steps+ steps_since_rev
    #print(f"Moving {abs(steps_since_rev)} steps in direction {direction}...")
    _log_homing_event(f"Moving {abs(steps_since_rev)} steps in direction {direction}")

    current_rev = rev_count
    
    # Move full revolutions first
    if rev_diff != 0:
        #print(f"Moving {abs(rev_diff)} full revs in direction {direction}...")
        _log_homing_event(f"Moving {abs(rev_diff)} full revs in direction {direction}")
        while True:
            if rev_count != (current_rev + (rev_diff * direction)):
                #move_stepper(seq, 1, step_delay, direction)
                move_biased(steps =1, step_delay=step_delay, direction=direction)
                homing_steps += 1
            else: 
                break
    #print(f"Total revolutions moved: {abs(rev_diff)}")
    _log_homing_event(f"Total revolutions moved: {abs(rev_diff)}")


    #move the pre-revs steps + hysteresis 
    slower_delay = 0.005  # Adjust for slower speed of hysteresis

    overshoot_steps = _legacy_steps(20)
    #move_stepper(seq, steps_till_rev+overshoot_steps, slower_delay, direction)
    move_biased(steps = steps_till_rev+overshoot_steps, step_delay=slower_delay, direction=direction)
    homing_steps = homing_steps + steps_till_rev + overshoot_steps
    print(f"Moving {abs(steps_till_rev + overshoot_steps)} steps in direction {direction} at slower speed...")
    
    # Move back beyond home by 60 steps in opposite direction, slower
    opposite_direction = -1* direction
    
    reverse_overshoot_steps = _legacy_steps(10)

    print(f"Reversing direction by {overshoot_steps} steps (60 steps total) at slower speed.")
    #move_stepper(seq, overshoot_steps + reverse_overshoot_steps, slower_delay, opposite_direction)
    move_biased(overshoot_steps + reverse_overshoot_steps, step_delay=slower_delay, direction=opposite_direction)
    homing_steps = homing_steps - (reverse_overshoot_steps)

    slowest_delay = slower_delay * 2

    #Slowly approach home position by moving forward 10 steps
    fine_tune_steps = _legacy_steps(10)
    #print(f"Final forward adjustment of {fine_tune_steps} steps at slowest speed.")
    #move_stepper(seq, fine_tune_steps, slowest_delay, direction)
    move_biased(fine_tune_steps, step_delay=slowest_delay, direction=direction)

    # Move remaining steps if not aligned
    remaining_steps = step_diff - homing_steps
    print(f"Remaining steps to home: {remaining_steps}")
    time.sleep(5)

    # If remaining steps is zero, we are already at home
    while remaining_steps != 0:
        if remaining_steps > 0:

            print(f"Moving {abs(remaining_steps)} steps in direction {direction}...")
            #move_stepper(seq, abs(remaining_steps), slowest_delay, direction)
            move_biased(abs(remaining_steps), step_delay=slowest_delay, direction=direction)
            homing_steps = homing_steps + remaining_steps

        if remaining_steps < 0:
            print(f"Moving {abs(remaining_steps)} steps in direction {direction}...")
            #move_stepper(seq, abs(remaining_steps), slowest_delay, -1*direction)
            move_biased(abs(remaining_steps), step_delay=slowest_delay, direction=-1*direction)
            homing_steps = homing_steps - remaining_steps
        # Recalculate remaining steps
        remaining_steps = step_count

    print("Arrived at home position!")


def set_home():
    global home_delta_steps, home_revs, home_steps, home_pre_rev_steps, step_count, rev_count, delta_steps, home, _seq_index
    global steps_till_rev, last_switch1_state, last_switch2_state , transition_sequence, steps_since_rev
    print("Please use jog mode to set worm gear to home (switch 2 open) quit jog mode.")
    time.sleep(3)
    drain_stdin()
    jog_mode2()
    drain_stdin()

    init_steps = step_count
    init_rev_count = rev_count
    init_delta_steps = delta_steps

    # print("Now that the large Worm Gear is set to open, move the motor using jog untill optical home is acheived, then quit jog mode")
    # time.sleep(4)
    # jog_mode2(step_delay=0.001)
    
    #home_delta_steps = steps_since_rev
    home_revs = rev_count
    home_steps = step_count
    #home_pre_rev_steps = steps_till_rev

    # Explicitly reset counts when setting new home
    step_count = 0  # Reset step count when explicitly setting home
    _seq_index = 0
    rev_count = 0   # Reset revolution count
    steps_since_rev = 0
    steps_till_rev = 0

    # persist current position to disk
    _persist_step_count()
    
    print('Total Steps, total revs, delta steps pre-rev, delta steps post-revs:', home_steps, ' ,' ,home_revs, ' ,',home_pre_rev_steps, ' ,',home_delta_steps )


def _sleep_with_cancel(seconds: float, check_dt: float = 0.05):
    """Sleep in small chunks so the cancel listener can interrupt."""
    if seconds <= 0:
        return
    t0 = time.time()
    while (time.time() - t0) < seconds:
        _check_cancel()
        time.sleep(min(check_dt, seconds))

def _frange_inclusive(a: float, b: float, step: float):
    """Inclusive float range that works for +/- direction."""
    if step == 0:
        raise ValueError("step must be non-zero")
    if (b - a) == 0:
        return [a]
    direction = 1 if (b > a) else -1
    step = abs(step) * direction

    vals = []
    x = a
    # Inclusive with tolerance to avoid float drift
    tol = abs(step) * 1e-6 + 1e-12
    if direction > 0:
        while x <= b + tol:
            vals.append(x)
            x += step
    else:
        while x >= b - tol:
            vals.append(x)
            x += step
    return vals

def _move_scan_to_first_target(target_steps: int, *, mode: str, force_home: bool = False):
    """
    Position to the first point of a scan.
    - `mode` only controls the very first acquisition of the scan start.
    - `force_home` is used by rehome-per-repeat.
    """
    target_steps = int(round(target_steps))

    if force_home or mode == "from_home_ccw":
        if not go_home2():
            raise RuntimeError("Could not establish Disk Home before scan start.")
        _stage_move_signed(target_steps)
        return

    if mode == "shortest":
        _move_to_target_steps_fixed_approach(target_steps)
        return

    raise ValueError("mode must be 'from_home_ccw' or 'shortest'")

def _scan_step_to_target(target_steps: int):
    """
    Move directly from the current point to the next scan point without re-homing.
    This preserves a true stepped scan between endpoints.
    """
    delta_steps = int(round(target_steps)) - int(round(step_count))
    if delta_steps != 0:
        _stage_move_signed(delta_steps)

def scan_wavelength(
    lam1: float,
    lam2: float,
    order_m: int,
    step_nm: float,
    *,
    mode: str = "shortest",
    repeats: int = 1,
    dwell_s: float = 0.0,
    margin_nm: float = 0.0,
    rehome_each_repeat: bool = False,
    pingpong: bool = False,
):
    """
    Scan from wavelength lam1 to lam2 (nm) in steps of step_nm.
    margin_nm expands the scan on both ends in the scan direction:
      - If increasing: start=lam1-margin, end=lam2+margin
      - If decreasing: start=lam1+margin, end=lam2-margin

    repeats repeats the scan.
    pingpong alternates direction each repeat.
    rehome_each_repeat will re-index at Disk Home before each repeat.
    dwell_s pauses at each point (useful for integrations).
    """
    if repeats < 1:
        raise ValueError("repeats must be >= 1")
    if order_m == 0:
        raise ValueError("order_m must be non-zero")
    if step_nm <= 0:
        raise ValueError("step_nm must be > 0")
    if dwell_s < 0:
        raise ValueError("dwell_s must be >= 0")
    if margin_nm < 0:
        raise ValueError("margin_nm must be >= 0")

    direction = 1 if lam2 > lam1 else -1
    start = lam1 - direction * margin_nm
    end   = lam2 + direction * margin_nm

    points_fwd = _frange_inclusive(start, end, step_nm)
    target_steps_fwd = []
    for lam in points_fwd:
        theta = _theta_deg_from_lambda(lam, order_m)
        target_steps_fwd.append(_target_steps_from_disk_home(theta))

    for r in range(repeats):
        _check_cancel()
        reverse_repeat = pingpong and (r % 2 == 1)
        points = list(reversed(points_fwd)) if reverse_repeat else points_fwd
        target_steps = list(reversed(target_steps_fwd)) if reverse_repeat else target_steps_fwd

        force_home = rehome_each_repeat
        if r == 0 or force_home:
            _move_scan_to_first_target(target_steps[0], mode=mode, force_home=force_home)
        else:
            _scan_step_to_target(target_steps[0])

        for i, lam in enumerate(points):
            _check_cancel()
            if i != 0:
                _scan_step_to_target(target_steps[i])

            # Optional dwell for acquisition
            _sleep_with_cancel(dwell_s)

def scan_angle(
    theta1_deg: float,
    theta2_deg: float,
    step_deg: float,
    *,
    mode: str = "shortest",
    repeats: int = 1,
    dwell_s: float = 0.0,
    margin_deg: float = 0.0,
    rehome_each_repeat: bool = False,
    pingpong: bool = False,
):
    """
    Scan from angle theta1 to theta2 (deg) in steps of step_deg.
    margin_deg expands the scan on both ends in the scan direction.
    """
    if repeats < 1:
        raise ValueError("repeats must be >= 1")
    if step_deg <= 0:
        raise ValueError("step_deg must be > 0")
    if dwell_s < 0:
        raise ValueError("dwell_s must be >= 0")
    if margin_deg < 0:
        raise ValueError("margin_deg must be >= 0")

    direction = 1 if theta2_deg > theta1_deg else -1
    start = theta1_deg - direction * margin_deg
    end   = theta2_deg + direction * margin_deg

    points_fwd = _frange_inclusive(start, end, step_deg)
    target_steps_fwd = [_target_steps_from_disk_home(theta) for theta in points_fwd]

    for r in range(repeats):
        _check_cancel()
        reverse_repeat = pingpong and (r % 2 == 1)
        points = list(reversed(points_fwd)) if reverse_repeat else points_fwd
        target_steps = list(reversed(target_steps_fwd)) if reverse_repeat else target_steps_fwd

        force_home = rehome_each_repeat
        if r == 0 or force_home:
            _move_scan_to_first_target(target_steps[0], mode=mode, force_home=force_home)
        else:
            _scan_step_to_target(target_steps[0])

        for i, th in enumerate(points):
            _check_cancel()
            if i != 0:
                _scan_step_to_target(target_steps[i])
            _sleep_with_cancel(dwell_s)

def scan_menu():
    """
    Interactive scan menu: wavelength scan or angle scan, with repeats and margin options.
    """
    while True:
        print("\n--- Scan Menu ---")
        print("1) Scan wavelength λ1 → λ2 (nm)")
        print("2) Scan angle θ1 → θ2 (deg)")
        print("Q) Back")

        ch = input("Select: ").strip().lower()
        if ch in ("q", "quit"):
            return

        try:
            mode = _ask_approach_mode()  # from_home_ccw or shortest (per your implementation)

            repeats_s = input("Repeats (ENTER=1): ").strip()
            repeats = int(repeats_s) if repeats_s else 1

            ping_s = input("Ping-pong direction each repeat? [y/N]: ").strip().lower()
            pingpong = (ping_s == "y")

            reh_s = input("Re-home at Disk Home at start of each repeat? [y/N]: ").strip().lower()
            rehome_each_repeat = (reh_s == "y")

            dwell_s_raw = input("Dwell at each point in seconds (ENTER=0): ").strip()
            dwell_s = float(dwell_s_raw) if dwell_s_raw else 0.0

            if ch == "1":
                lam1 = float(_prompt_or_quit("Start wavelength λ1 (nm) (q=quit): "))
                lam2 = float(_prompt_or_quit("End wavelength   λ2 (nm) (q=quit): "))
                m = int(_prompt_or_quit("Diffraction order m (int) (q=quit): "))

                step_nm_raw = _prompt_or_quit("Step size (nm) (ENTER=1.0, q=quit): ")
                step_nm = float(step_nm_raw) if step_nm_raw else 1.0
                margin_nm_raw = input("Endpoint margin (nm) (ENTER=0): ").strip()
                margin_nm = float(margin_nm_raw) if margin_nm_raw else 0.0

                print("Scanning... (press 'q' or Esc to cancel)")
                _run_cancellable(
                    scan_wavelength,
                    lam1, lam2, m, step_nm,
                    mode=mode,
                    repeats=repeats,
                    dwell_s=dwell_s,
                    margin_nm=margin_nm,
                    rehome_each_repeat=rehome_each_repeat,
                    pingpong=pingpong,
                )

            elif ch == "2":
                th1 = float(_prompt_or_quit("Start angle θ1 (deg) (q=quit): "))
                th2 = float(_prompt_or_quit("End angle   θ2 (deg) (q=quit): "))
                step_deg_raw = _prompt_or_quit("Step size (deg) (ENTER=0.05, q=quit): ")
                step_deg = float(step_deg_raw) if step_deg_raw else 0.05
                margin_deg_raw = input("Endpoint margin (deg) (ENTER=0): ").strip()
                margin_deg = float(margin_deg_raw) if margin_deg_raw else 0.0

                print("Scanning... (press 'q' or Esc to cancel)")
                _run_cancellable(
                    scan_angle,
                    th1, th2, step_deg,
                    mode=mode,
                    repeats=repeats,
                    dwell_s=dwell_s,
                    margin_deg=margin_deg,
                    rehome_each_repeat=rehome_each_repeat,
                    pingpong=pingpong,
                )
            else:
                print("Invalid option.")
                continue

        except _QuitToMain:
            print("Leaving Scan menu.")
            return
        except ValueError as e:
            print(f"Input error: {e}")
            continue


def home_menu():
    while True:
        print("\n--- Home Menu ---")
        print("1: Go to Disk Home")
        print("2: Reset Disk Home (redefine Disk Home)")
        print("3: Set Optical Home Offset (interactive)")
        print("4: Go to Optical Home (Disk Home + saved offset)")
        print("Q: Return to main menu")
        choice = input("Choose an option (1/2/3/4/Q): ").strip().lower()

        if choice == '1':
            print("Moving to Disk Home... (press 'q' or Esc to cancel)")
            _run_cancellable(go_home2)     # ← wrap it
            break

        elif choice == '2':
            try:
                confirm = _prompt_or_quit("Are you sure you want to reset Disk Home? (y = yes, q=quit): ")
            except _QuitToMain:
                print("Exiting home menu."); break
            if confirm.strip().lower() == 'y':
                _run_cancellable(set_home)  # optional wrap
                break
            else:
                print("Reset cancelled. Returning to home menu.")

        elif choice == '3':
            # already has its own key handler; it's interactive & cancellable with 'q'
            find_optical_home_offset()

        elif choice == '4':
            print("Going to Optical Home... (press 'q' or Esc to cancel)")
            _run_cancellable(go_optical_home)  # ← wrap it
            break

        elif choice == 'q':
            print("Exiting home menu.")
            break
        else:
            print("Invalid option. Please enter 1, 2, 3, 4, or Q.")


def position_menu():
    while True:
        print("\n--- Position Menu ---")
        print("1) Calibrate S2 360° (steps/degree)")
        print("2) Move to angle θ (degrees)")
        print("3) Move to wavelength λ (nm) & order m")
        print("4) Show current S2 steps/deg and integrity check")
        print("5) Wavelength calibration (refs & fit)") 
        print("J) Jog mode (UP/DOWN move, LEFT/RIGHT adjust speed)")
        print("H) Go Home")
        print("Q) Back to main menu")

        choice = input("Select: ").strip().lower()

        if choice == "1":
            dirch = input("Approach S2 edge in direction [+/-] (ENTER=+): ").strip()
            approach_dir = -1 if dirch == "-" else +1
            #_run_cancellable(calibrate_s2_steps_per_rev, approach_dir=approach_dir)  # optional

            steps = calibrate_s2_steps_per_rev(approach_dir=approach_dir)
            # (S2 calibration is already persisted by calibrate_s2_steps_per_rev)
            if not go_home2():
                print("[Pos] S2 calibration finished, but re-home failed afterward.")

        elif choice == "2":
            try:
                theta_s = _prompt_or_quit("Enter target angle θ in degrees (q=quit): ")
                theta = float(theta_s)
                mode = _ask_approach_mode()   # may raise _QuitToMain
            except _QuitToMain:
                print("Leaving Position menu."); return
            except ValueError:
                print("θ must be a number."); continue

            print("Moving... (press 'q' or Esc to cancel)")
            _run_cancellable(move_to_angle_deg, theta, mode=mode)
            ok, err = check_step_integrity()
            print(f"Integrity: {'OK' if ok else 'DRIFT'} (Δ={err} steps)  | step_count={step_count}")

        elif choice == "3":
            try:
                lam_s = _prompt_or_quit("Enter wavelength λ in nm (q=quit): ")
                lam = float(lam_s)
                m_s   = _prompt_or_quit("Enter diffraction order m (integer) (q=quit): ")
                m   = int(m_s)
                mode = _ask_approach_mode()   # may raise _QuitToMain
            except _QuitToMain:
                print("Leaving Position menu."); return
            except ValueError:
                print("λ must be number, m must be integer."); continue

            print("Moving... (press 'q' or Esc to cancel)")

            #using the m*lambda=dsin(theta) function
            #_run_cancellable(move_to_wavelength, lam, m, mode=mode)

            #using the m*lambda=d(sin(alpa)+sin(beta)) function
            _run_cancellable(move_to_wavelength_nonlinear, lam, m, mode=mode)


            ok, err = check_step_integrity()
            print(f"Integrity: {'OK' if ok else 'DRIFT'} (Δ={err} steps)  | step_count={step_count}")

        elif choice == "4":
            _ensure_steps_per_deg()
            print(f"S2_STEPS_PER_REV={S2_STEPS_PER_REV}, STEPS_PER_DEG={STEPS_PER_DEG:.6f}")
            ok, err = check_step_integrity()
            print(f"Integrity: {'OK' if ok else 'DRIFT'} (Δ={err} steps)  | step_count={step_count}")

        elif choice == "5":
            wl_cal_menu()

        elif choice in ("j", "J"):
            drain_stdin()
            jog_mode2()
            drain_stdin()
            # returns here when you quit Jog mode

        elif choice in ("h", "H"):
            _run_cancellable(go_optical_home) 

        elif choice in ("q", "Q"):
            print("Leaving Position menu.")
            return
        else:
            print("Invalid option.")


def _stage_move_signed(delta_steps):
    """
    Signed move with a fast pre-roll when far, then slower finishing.
    +delta => CW, -delta => CCW.
    """
    if delta_steps == 0:
        return
    direction = 1 if delta_steps > 0 else -1
    remaining = abs(delta_steps)

    print(f"[Stage] Moving {remaining} steps, dir={'CW' if direction>0 else 'CCW'}")

    # fast chunk
    if remaining > FAR_STEP_THRESHOLD_MOVE:
        fast_chunk = remaining - FAR_STEP_THRESHOLD_MOVE
        move_biased(fast_chunk, FAST_STEP_DELAY, direction)
        remaining -= fast_chunk

    # finish at moderate speed
    if remaining > 0:
        move_biased(remaining, EDGE_STEP_DELAY, direction)


# --- Quick quit-from-any-prompt plumbing ---
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



def goto_menu():
    """
    Repeatedly prompt for either a relative or absolute step move.
    +N/-N => relative CW/CCW move.
    =N    => absolute step target from Disk Home.
    Type 'j' to jump to Jog, 'q' to quit back to main.
    """
    global step_delay  # for jog_mode2

    print("\n--- GoTo (steps) ---")
    print(f"Current step_count = {step_count}")
    print("Enter either:")
    print("  +N = relative CW move")
    print("  -N = relative CCW move")
    print("  =N = absolute target step position")
    print("  j  = jump to Jog")
    print("  q  = quit to main menu")

    while True:
        s = input("GoTo command (+/-N relative, =N absolute, j, q): ").strip().lower()
        if s == "q":
            print("Leaving GoTo.")
            return
        if s == "j":
            drain_stdin()
            jog_mode2()   # returns here when you quit Jog
            drain_stdin()
            continue
        if s.startswith("="):
            try:
                target_steps = int(s[1:].strip())
            except ValueError:
                print("Absolute targets must look like =500 or =-1200.")
                continue
            delta_steps = target_steps - step_count
            if delta_steps == 0:
                print(f"Already at absolute step {target_steps}.")
                continue
            print(
                f"[GoTo] Absolute target: {target_steps} steps "
                f"(delta {delta_steps:+d}, {'CW' if delta_steps > 0 else 'CCW'})"
            )
            _stage_move_signed(delta_steps)
            print(f"Done. Current step_count = {step_count}")
            continue

        try:
            user_steps = int(s)
        except ValueError:
            print("Please enter +N/-N, =N, j, or q.")
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

    # Optional: trim on startup
    try:
        enforce_log_size_limit(LOG_FILE, max_bytes=10*1024*1024)
    except Exception as e:
        print(f"[LogTrim] Startup trim skipped: {e}")

    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    fmt = logging.Formatter('%(asctime)s - %(message)s')

    # Use our tail-truncating handler
    fh = TailTruncatingFileHandler(
        LOG_FILE,
        mode='a',
        max_bytes=10 * 1024 * 1024,  # 10 MB cap
        keep_bytes=8 * 1024 * 1024,  # keep last ~8 MB on trim
        check_every=20               # check size every 20 emits
    )
    fh.setFormatter(fmt)

    logger.addHandler(fh)
    if LOG_TO_CONSOLE:
        ch = logging.StreamHandler()
        ch.setFormatter(fmt)
        logger.addHandler(ch)

    print(f"[Log] Logging to {LOG_FILE}")
    return LOG_FILE

def _persist_step_count():
    """Atomically write current step_count to STEP_FILE."""
    global _last_persist_step, _last_persist_time
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = STEP_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(str(step_count))
    os.replace(tmp, STEP_FILE)  # atomic on POSIX/macOS/Windows NTFS
    _last_persist_step = int(step_count)
    _last_persist_time = time.time()

def _maybe_persist_step_count(force=False):
    now = time.time()
    if force:
        _persist_step_count()
        return
    if abs(step_count - _last_persist_step) >= PERSIST_EVERY_STEPS:
        _persist_step_count()
        return
    if (now - _last_persist_time) >= PERSIST_EVERY_SECONDS:
        _persist_step_count()

def _load_persisted_step_count():
    """Initialize step_count from STEP_FILE if present."""
    global step_count, _last_persist_step, _last_persist_time, _seq_index
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
    _seq_index = _seq_index_from_step_count(step_count)
    _last_persist_step = int(step_count)
    _last_persist_time = time.time()

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

def initialize_runtime(*, show_banner=True):
    """
    Initialize hardware/runtime state once so the backend can be safely imported by a GUI.
    """
    global _runtime_initialized, step_delay, S2_STEPS_PER_REV, STEPS_PER_DEG
    global BACKLASH_STEPS, DIR_REVERSE_SCALE, steps_per_rev, cal, WL, _cal

    with _runtime_init_lock:
        if _runtime_initialized:
            return build_state_snapshot()

        _init_board()
        WL = load_wl_cal(STATE_DIR)   # {"refs":[...], "model": {...} or None}
        wait_for_initial_sensor_states()
        setup_logging()
        _load_persisted_step_count()
        _load_persisted_jog_speed()
        _load_optical_offset()

        cal = load_cal(STATE_DIR) or {}
        apply_cal_to_globals(cal, globals())
        _upgrade_legacy_calibration_units()

        _cal = cal
        _cal.setdefault("wl_refs", [])
        _cal.setdefault("wl_model", None)

        if globals().get("STEPS_PER_DEG") in (None, 0):
            S2_STEPS_PER_REV = _s2_steps_fallback()
            STEPS_PER_DEG = S2_STEPS_PER_REV / 360.0
            print(f"[Cal] Using fallback S2_STEPS_PER_REV={S2_STEPS_PER_REV} -> STEPS_PER_DEG={STEPS_PER_DEG:.6f}")
        else:
            print(
                f"[Cal] Loaded: S2_STEPS_PER_REV={S2_STEPS_PER_REV}, "
                f"STEPS_PER_DEG={STEPS_PER_DEG:.6f}, BACKLASH={BACKLASH_STEPS}, "
                f"REVERSE_SCALE={DIR_REVERSE_SCALE}"
            )

        if steps_per_rev is not None and steps_per_rev < 200:
            print("[Warn] This controller now counts one electrical half-step per software step.")
            print("[Warn] Existing S1/S2 calibration appears to be from the old full-sequence counting scheme.")
            print("[Warn] Re-run S1, S2, backlash, and optical-home calibration before trusting positions.")

        if show_banner:
            print("[Log] Starting stepper motor control")

        _runtime_initialized = True
        return build_state_snapshot()

def build_state_snapshot(*, order_m=1):
    if board is None:
        return {
            "step_count": int(step_count),
            "theta_deg": None,
            "estimated_wavelength_nm": None,
            "s1": "Disconnected",
            "s2": "Disconnected",
            "jog_speed_ms": _delay_to_ms(step_delay),
            "optical_offset_steps": int(OPTICAL_HOME_OFFSET_STEPS),
            "home_window_open": False,
            "debug_enabled": bool(DEBUG_MOTION_LOGGING),
        }

    s1 = _sensor_state_label(board.digital[optical_pin].read())
    s2 = _sensor_state_label(board.digital[optical_pin2].read())
    theta_deg = None
    try:
        theta_deg = float(_steps_to_theta_deg(step_count))
    except Exception:
        theta_deg = None

    wavelength_nm = None
    try:
        wl_model = _normalize_wl_model(_wl_load(STATE_DIR).get("model"))
        wavelength_nm = float(_wl_model_value(theta_deg or 0.0, wl_model) * order_m)
    except Exception:
        wavelength_nm = None

    return {
        "step_count": int(step_count),
        "theta_deg": theta_deg,
        "estimated_wavelength_nm": wavelength_nm,
        "s1": s1,
        "s2": s2,
        "jog_speed_ms": _delay_to_ms(step_delay),
        "optical_offset_steps": int(OPTICAL_HOME_OFFSET_STEPS),
        "home_window_open": bool(s2 == "OPEN"),
        "debug_enabled": bool(DEBUG_MOTION_LOGGING),
    }

class MonochromatorGUIBackend:
    """
    Thin, thread-safe facade over the copied controller logic for use by a PySide6 GUI.
    """
    def __init__(self):
        self._lock = threading.RLock()

    def initialize(self):
        with self._lock:
            return initialize_runtime(show_banner=False)

    def status(self):
        with self._lock:
            initialize_runtime(show_banner=False)
            return build_state_snapshot()

    def cancel(self):
        _set_cancel(True)

    def _run(self, fn, *args, **kwargs):
        with self._lock:
            initialize_runtime(show_banner=False)
            _set_cancel(False)
            try:
                fn(*args, **kwargs)
            except _QuitToMain as e:
                raise RuntimeError("Operation cancelled") from e
            finally:
                _set_cancel(False)
            return build_state_snapshot()

    def set_jog_speed_ms(self, ms):
        with self._lock:
            initialize_runtime(show_banner=False)
            _set_step_delay_ms(ms, persist=True)
            return build_state_snapshot()

    def jog_step(self, direction, steps=1):
        steps = int(steps)
        if steps <= 0:
            raise ValueError("steps must be > 0")
        return self._run(move_exact, steps, step_delay, 1 if int(direction) > 0 else -1)

    def move_relative_steps(self, delta_steps):
        return self._run(_stage_move_signed, int(delta_steps))

    def move_absolute_steps(self, target_steps):
        target_steps = int(target_steps)
        with self._lock:
            initialize_runtime(show_banner=False)
            delta = target_steps - int(step_count)
        return self._run(_stage_move_signed, delta)

    def move_to_angle(self, theta_deg, mode="from_home_ccw"):
        return self._run(move_to_angle_deg, float(theta_deg), mode)

    def move_to_wavelength(self, wavelength_nm, order_m, mode="from_home_ccw"):
        return self._run(move_to_wavelength_nonlinear, float(wavelength_nm), int(order_m), mode)

    def go_disk_home(self):
        return self._run(go_home2)

    def go_optical_home(self):
        return self._run(go_optical_home)

    def scan_wavelength(self, lam1, lam2, order_m, step_nm, **kwargs):
        return self._run(
            scan_wavelength,
            float(lam1),
            float(lam2),
            int(order_m),
            float(step_nm),
            **kwargs,
        )

    def scan_angle(self, theta1_deg, theta2_deg, step_deg, **kwargs):
        return self._run(
            scan_angle,
            float(theta1_deg),
            float(theta2_deg),
            float(step_deg),
            **kwargs,
        )

    def shutdown(self):
        global board, it, port, switch1, switch2, last_states, _runtime_initialized
        _set_cancel(True)
        with self._lock:
            if board is None:
                _runtime_initialized = False
                return

            for pin in pins:
                try:
                    board.digital[pin].write(0)
                except Exception:
                    pass
            try:
                board.exit()
            except Exception:
                pass
            finally:
                board = None
                it = None
                port = None
                switch1 = None
                switch2 = None
                last_states = [None, None]
                _runtime_initialized = False


def main():
    initialize_runtime(show_banner=True)

    print("Stepper Motor Control")
    print("G: GoTo Step. +N Forward CW, -N Reverse CCW")
    print("J: Jog Mode (UP/DOWN move, LEFT/RIGHT adjust speed)")
    print("D: Toggle Debug Mode")
    print("V: Speed Settings")
    print("H: Home Menu")
    print("C: Calibration")
    print("P: Position Menu (Angle/Wavelength)")
    print("S: Scan Menu (wavelength/angle sequences)")
    print("Q: Quit")

    while True:
        cmd = prompt_cmd("Enter option GoTo, Jog, Debug, Speed Settings, Home, Calibration, Position Menu, Scan, Quit (G/J/D/V/H/C/P/S/Q): ")
        if cmd in ('G','g'):
           goto_menu()
           drain_stdin()
        elif cmd in ('J', 'j'):
            jog_mode2()
            drain_stdin()
        elif cmd in ('D', 'd'):
            toggle_debug_mode()
            drain_stdin()
        elif cmd in ('V', 'v'):
            step_delay = set_speed(step_delay)
            drain_stdin()
        elif cmd in ('H','h'): 
            home_menu()
            drain_stdin()
        elif cmd in ('C','c'):
            print("Starting Calibration")
            calibration()
            drain_stdin()
        elif cmd in ('P','p'):  
            position_menu()
            drain_stdin()

            #move_to_wavelength(532, 1, mode="from_home_ccw")   # green, 1st order, repeatable approach
            # or
            #move_to_wavelength(650, 1, mode="shortest")        # quicker, chooses direction automatically
            #move_to_angle_deg(12.5, mode="from_home_ccw").     #going to 12.5 degrees, counterclockwise from home
        elif cmd in ('S','s'):
            scan_menu()
                # example: scan_wavelength(500, 700, m=1, step_nm   = 5, mode="shortest", repeats=2, dwell_s=0.5, margin_nm=10, rehome_each_repeat=True, pingpong=True)
                # example: scan_angle(0, 90, step_deg=5, mode="from_home_ccw", repeats=1, dwell_s=0.5, margin_deg=5, rehome_each_repeat=False, pingpong=False)  
            drain_stdin()

        elif cmd in ('q', 'Q'):
            _clear_sensor_terminal_line()
            print("Quitting.")
            settle_terminal_input()
            break
        else:
            print("Invalid option. Enter G, J, D, V, H, C, P, S, or Q.")
            drain_stdin()
    for pin in pins:
        board.digital[pin].write(0)
    board.exit()

if __name__ == '__main__':
    main()  

# try 587.5 nm at m=1, 686.7, 501.5 nm
