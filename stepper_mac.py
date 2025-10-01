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

# used to setup logging and control log file size
from log_utils import TailTruncatingFileHandler, enforce_log_size_limit


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
REHOME_BUMP_STEPS  = 100   # how far to leave the OPEN window when already at home

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


# cancellimng functions
CANCEL = False

def _set_cancel(v=True):
    global CANCEL
    CANCEL = v

def _check_cancel():
    # Raise to unwind back to menus
    if CANCEL:
        raise _QuitToMain

def _start_cancel_listener(hint="Press 'q' or Esc to cancel and return to the main menu."):
    print(hint)
    _set_cancel(False)

    def on_press(key):
        try:
            if key == keyboard.Key.esc:
                _set_cancel(True); return False
            if hasattr(key, "char") and key.char and key.char.lower() == 'q':
                _set_cancel(True); return False
        except Exception:
            pass  # never let listener crash

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    return listener

def _stop_cancel_listener(listener):
    try:
        if listener: listener.stop()
    finally:
        _set_cancel(False)

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

def angle_to_steps(theta_deg):
    _ensure_steps_per_deg()
    direction = -1 #if CW is positive and 1 if CW is negative
    steps = int(round(direction*theta_deg * STEPS_PER_DEG))
    return steps

def _stage_move_signed(delta_steps):
    """
    Signed move with a fast pre-roll when far, then slower finishing.
    """
    if delta_steps == 0:
        return
    direction = 1 if delta_steps > 0 else -1
    remaining = abs(delta_steps)

    # fast chunk
    if remaining > FAR_STEP_THRESHOLD_MOVE:
        fast_chunk = remaining - FAR_STEP_THRESHOLD_MOVE
        move_biased(fast_chunk, FAST_STEP_DELAY, direction)
        remaining -= fast_chunk

    # finish at moderate speed
    if remaining > 0:
        move_biased(remaining, EDGE_STEP_DELAY, direction)

def move_to_angle_deg(theta_deg, mode="from_home_ccw"):
    """
    mode:
      - "from_home_ccw": go_home2() first, then move CCW (+ steps) to target.
                         If target needs CW (negative steps), you can choose:
                           (a) allow CW (faster) OR (b) wrap +360° (always CCW).
                         Below I allow BOTH directions by default; if you want
                         single-direction-only repeatability, set force_ccw=True.
      - "shortest": move from current position in whichever direction is shorter.
    """
    # 1) target steps from HOME (m=0)
    target_steps = angle_to_steps(theta_deg)

    if mode == "from_home_ccw":
        # Option A: always start from a repeatable reference
        go_home2()

        # change this flag to True if you want single-direction approach only:
        force_ccw = False

        if force_ccw and target_steps < 0:
            # wrap to positive equivalent (adds one full turn)
            _ensure_steps_per_deg()
            target_steps = (target_steps % S2_STEPS_PER_REV)

        _stage_move_signed(target_steps)

        # tiny settle (take out micro-backlash)
        #move_biased(3, CENTER_STEP_DELAY, +1 if target_steps>=0 else -1)
        #move_biased(3, CENTER_STEP_DELAY, -1 if target_steps>=0 else +1)

    elif mode == "shortest":
        # Move from wherever we are now.
        # We need current "from-home steps"; reuse your step_count (motor axis).
        # If you prefer reference from S2, you can track a separate "s2_step_count".
        current_steps_from_home = step_count  # using your convention (CW positive)
        delta = target_steps - current_steps_from_home
        # normalize delta to shortest path around a circle if a full 360 wrap is allowed
        _ensure_steps_per_deg()
        if abs(delta) > S2_STEPS_PER_REV / 2:
            # take the shorter wrap path
            if delta > 0:
                delta -= S2_STEPS_PER_REV
            else:
                delta += S2_STEPS_PER_REV
        _stage_move_signed(delta)

    else:
        raise ValueError("mode must be 'from_home_ccw' or 'shortest'")

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

def check_step_integrity(tolerance_steps=4):
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
        if BACKLASH_STEPS > 0:
            # Take up slack. These are "compensation" steps; you can treat them
            # as physical-only, but since your counters track actual motion,
            # letting them count is fine.
            move_stepper(seq, BACKLASH_STEPS, step_delay, direction)

    # Scale reverse steps for steady-state bias
    adj_steps = steps
    if direction == -1 and DIR_REVERSE_SCALE != 1.0:
        # Keep fractional error small via accumulator
        _rev_scale_err_accum += steps * DIR_REVERSE_SCALE
        adj_steps = int(round(_rev_scale_err_accum))
        _rev_scale_err_accum -= adj_steps

        # Ensure at least one step if nonzero commanded
        if steps > 0 and adj_steps == 0:
            adj_steps = 1

    move_stepper(seq, adj_steps, step_delay, direction)
    _last_move_dir = direction
    _check_cancel() 


def move_stepper(seq, steps,step_delay, direction):
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home, switch1, switch2, steps_per_rev, steps_till_rev, steps_since_rev, is_1rev_completed
    # do NOT reinitialize them here!
    # just update their values as you go

    if direction == -1:
        sequence = list(reversed(seq))
    else: sequence = seq
    

    for _ in range(steps):
        _check_cancel()
        for step in sequence:
            for pin, val in zip(pins, step):
                board.digital[pin].write(val)
            monitor_sensors()

             # Read switch 1 (smaller disk) state each time you move a step
            switch1 = board.digital[optical_pin].read()

            if switch1 is None:
                state1 = "WAITING"
            elif switch1:
                state1 = "BLOCKED"
            else:
                state1 = "OPEN"

            if state1 == "OPEN":
                is_1rev_completed = True

            # Track transition sequence for open->blocked->open
            if last_switch1_state is not None:
                # Only add when state changes
                if state1 != last_switch1_state:
                    transition_sequence.append(state1)
                    # Keep last three states only
                    if len(transition_sequence) > 3:
                        transition_sequence.pop(0)

                    # Check for open->blocked->open
                    if transition_sequence == ["OPEN", "BLOCKED", "OPEN"]:
                         #if steps_since_rev > steps_per_rev * 0.8:  # Assuming at least 80% of expected steps before counting a rev
                        rev_count += 1 * direction
                        is_1rev_completed = True
                        steps_since_rev = 0
                        transition_sequence = ["OPEN"] # reset for next revolution

            last_switch1_state = state1
            time.sleep(step_delay)

        step_count += 1*direction  # After one complete seq, count as one step

        steps_since_rev += 1*direction

        #add persistence per step, remove if run time increases too much 
        _persist_step_count()

        if is_1rev_completed == False and state1 == "BLOCKED":
            steps_till_rev += 1 *direction

    # Turn all pins off when done
    for pin in pins:
        board.digital[pin].write(0)

    # persist current position to disk
    _persist_step_count()

    #for testing    
    print("step count", step_count)
    print("rev count", rev_count)

def set_speed(current_delay):
    print(f"Current speed: {int(current_delay*1000)} ms per step")
    print("Increasing ms/step results in slower rotation, with 1ms being fastest")
    while True:
        try:
            val = input("Enter new speed in ms per step (e.g., 15), or ENTER to keep: ").strip()
            if val == '':
                print("Keeping current speed.")
                return current_delay
            ms = int(val)
            if ms < 1 or ms > 1000:
                print("Enter a value between 1 and 1000 ms.")
                continue
            new_delay = ms / 1000.0
            print(f"Speed set to {ms} ms per step.")
            return new_delay
        except ValueError:
            print("Invalid input. Enter a number.")

def jog_mode(step_delay):
    global step_count, rev_count, home

    print("\n--- Jog Mode ---")
    print("Hold UP for forward CW, DOWN for reverse CCW. Press Q to quit jog mode.")
    time.sleep(2)
    try:
        while True:
            if keyboard.is_pressed('q'):
                print("Exiting jog mode.\n")
                break
            elif keyboard.is_pressed('up'):
                #move_stepper(seq, steps=1, step_delay =0.001, direction= 1)
                move_biased(steps = 1, step_delay=0.001, direction=1)
                 #for testing    
                print("step count", step_count)
                print("rev count", rev_count)

            elif keyboard.is_pressed('down'):
                # Move one step in reverse
                #move_stepper(seq, steps =1, step_delay =0.001, direction= -1)
                move_biased(steps =1, step_delay=0.001, direction=-1)
                 #for testing    
                print("step count", step_count)
                print("rev count", rev_count)
            else:
                # Release all pins to stop holding torque
                for pin in pins:
                    board.digital[pin].write(0)
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        for pin in pins:
            board.digital[pin].write(0)

def flush_input():
    termios.tcflush(sys.stdin, termios.TCIFLUSH)

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

def calibrate_directional_bias(pin=optical_pin, approach_dir=1, verbose=True):
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
        print("=== Directional Bias Calibration ===")
        print(f"Forward steady rev steps:        {f_steps}")
        print(f"Reverse first rev (with backlash): {r_first}")
        print(f"Reverse steady rev steps:         {r_steady}")
        print(f"-> BACKLASH_STEPS:               {BACKLASH_STEPS}")
        print(f"-> DIR_REVERSE_SCALE:            {DIR_REVERSE_SCALE:.6f}")
        if abs(DIR_REVERSE_SCALE - 1.0) < 0.01:
            print("Note: Reverse scale ~1.0 (little steady-state directional bias).")
        else:
            print("Note: Non-unity reverse scale detected (steady-state bias).")

    return {
        "forward_steps": f_steps,
        "reverse_first": r_first,
        "reverse_steady": r_steady,
        "backlash_steps": BACKLASH_STEPS,
        "reverse_scale": DIR_REVERSE_SCALE,
    }


def calibration():
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home, steps_per_rev, revs_per_rotation, steps_since_rev
    print("\n--- Calibration ---")
    print("Press 1 to calibrate the SMALL disk (optical switch 1).")
    print("Press 2 to calibrate the LARGE worm gear disk (optical switch 2).")
    print("Press 3 to calibrate directional bias/backlash on SMALL disk (optical switch 1).")
    print("Press Q to quit back to main menu.")

    while True:
        choice = input("Select calibration: 1 (small disk), 2 (large disk), Q (quit): ").strip().lower()
        if choice == '1':
            # --- Steps per rev FORWARD ---
            print("Calibrating steps per revolution (SMALL disk, FORWARD)...")
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

            break

        elif choice == '2':
            print("Calibrating worm gear...")
            print("Please use jog mode to set worm gear to home, quit jog mode, then run calibration steps as needed.")
            time.sleep(3)
            jog_mode2(step_delay=0.001)
            print("Big gear homed, begining calibration")

            calibration_step_counter =0 
            initial_rev_count = rev_count
            calibration_rev=0 

            transition_sequence2 = ["OPEN"]
            last_switch2_state = "OPEN"
            found_blocked = False

            while True:
            # Step forward once
                move_stepper(seq, 1, step_delay=0.001, direction=1)
                calibration_step_counter += 1 
                if rev_count - initial_rev_count > 0:
                    calibration_rev = rev_count-initial_rev_count
            # Check optical switch 2 state
                switch2 = board.digital[optical_pin2].read()
                if switch2 is None:
                    state2 = "WAITING"
                elif switch2:
                    state2 = "BLOCKED"
                else:
                    state2 = "OPEN"
            
            # Track transition: OPEN -> BLOCKED -> OPEN
                # if last_switch2_state is not None:
                #     if state2 != last_switch2_state:
                #         transition_sequence.append(state2)
                #         if len(transition_sequence) > 3:
                #             transition_sequence.pop(0)
                #         if transition_sequence == ["OPEN", "BLOCKED", "OPEN"]:
                #             print("Completed one revolution of worm gear!")
                #             break

                state2 = "BLOCKED" if board.digital[optical_pin2].read() else "OPEN"
                if not found_blocked and state2 == "BLOCKED":
                    found_blocked = True
                if found_blocked and state2 == "OPEN":
                    print("Completed one revolution (open->blocked->open) on worm gear.")
                    break
                last_switch2_state = state2  # Update for next loop

            print('Number of revs and steps: ', calibration_rev , ' revs , ' ,steps_since_rev, ' steps.')
            print('Number of total steps: ', calibration_step_counter)
            print('Resetting worm gear back home, stepping in reverse')
            time.sleep(5)
            move_stepper(seq, calibration_step_counter, step_delay=0.001, direction= -1)
            print('Worm gear back home, Number of total steps: ', calibration_step_counter)

        elif choice == '3':
            print("Calibrating directional bias/backlash on small disk (sensor 1)...")
            result = calibrate_directional_bias(pin=optical_pin, approach_dir=+1, verbose=True)


        elif choice == 'q' or 'Q':
            print("Exiting calibration menu.\n")
            return

        else:
            print("Invalid input. Enter 1, 2, or Q.")

    return



def jog_mode2(step_delay):
    print("\n--- Jog Mode ---")
    print("Hold UP for forward, DOWN for reverse. Press Q to quit jog mode.")

    running = True
    forward_pressed = False
    reverse_pressed = False

    def on_press(key):
        nonlocal running, forward_pressed, reverse_pressed
        if key == keyboard.Key.up:
            forward_pressed = True
        elif key == keyboard.Key.down:
            reverse_pressed = True
        elif hasattr(key, 'char') and key.char and key.char.lower() == 'q':
            running = False
            print("Exiting jog mode.")   
            print("step count", step_count)
            print("rev count", rev_count)
            return False

    def on_release(key):
        nonlocal forward_pressed, reverse_pressed
        if key == keyboard.Key.up:
            forward_pressed = False
        elif key == keyboard.Key.down:
            reverse_pressed = False

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while running:
        step_delay = 0.001  # Set a fixed step delay for responsiveness
        if forward_pressed:
            #move_stepper(seq, steps=1, step_delay=step_delay, direction=1)
            move_biased(steps =1, step_delay=step_delay, direction=1)
            while forward_pressed:
                #move_stepper(seq, steps=1, step_delay=step_delay, direction=1)
                move_biased(steps =1, step_delay=step_delay, direction=1)

        if reverse_pressed:
            #move_stepper(seq, steps=1, step_delay=step_delay, direction=-1)
            move_biased(steps =1, step_delay=step_delay, direction=-1)
            while reverse_pressed:
                #move_stepper(seq, steps=1, step_delay=step_delay, direction=-1)
                move_biased(steps =1, step_delay=step_delay, direction=-1)

        monitor_sensors()
        time.sleep(0.01)  # Adjust for responsiveness/smoothness

    listener.stop()

def monitor_sensors():
    global step_count, rev_count, switch1, switch2, last_states, home 
    
    switch1 = board.digital[optical_pin].read()
    switch2 = board.digital[optical_pin2].read()

    state1 = (
        "Waiting" if switch1 is None else
        "BLOCKED" if switch1 else
        "OPEN"
    )
    state2 = (
        "Waiting" if switch2 is None else
        "BLOCKED" if switch2 else
        "OPEN"
    )
 
    #print(f"Sensor 1: {state1} | Sensor 2: {state2}")
    logging.info(f"Sensor 1: {state1} | Sensor 2: {state2}")

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

def _read_open_filtered(pin, retries=4, sleep_s=0.001, default=False):
    """Return True if OPEN, False if BLOCKED. Debounce None->last-known. """
    val = None
    last = _last_s1_open if pin == optical_pin else default
    for _ in range(retries):
        v = board.digital[pin].read()
        if v is not None:
            val = not bool(v)   # pyfirmata: True means HIGH -> treats HIGH as BLOCKED
            break
        time.sleep(sleep_s)
    if val is None:
        return last if last is not None else default
    return val

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


#edge + step based homing function, as opposed to the only step based homing function gohome() 
# def go_home2():
#     """
#     Rehome with awareness of current sensor state.

#     If S2 is already OPEN (likely already at/near home), do a *slow* edge capture
#     based on the signed step_count (leave OPEN -> hit BLOCKED, then capture the
#     next BLOCKED->OPEN edge), then center and micro-tweak.

#     Otherwise (S2 not open), keep the existing fast-pre-roll + index/center.
#     """
#     global step_count

#     # Pick a direction from the sign of step_count (your convention: + => CW, - => CCW)
#     if step_count > 0:
#         approach_dir = -1
#     elif step_count < 0:
#         approach_dir = 1
#     else:
#         approach_dir = +1  # arbitrary default when zero

#     s2_is_open = not _read_blocked(optical_pin2)

#     if s2_is_open:
#         # --- Rehoming while already in S2's OPEN window ---
#         # 1) Leave OPEN to BLOCKED (slow)
#         _step_until_state(optical_pin2, True,  approach_dir, EDGE_STEP_DELAY)
#         # 2) Now capture the BLOCKED->OPEN rising edge (slow)
#         _step_until_state(optical_pin2, False, approach_dir, EDGE_STEP_DELAY)
#         # 3) Center on the window (do NOT use a fast first pass)
#         try:
#             open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=False)
#             print(f"[Rehome] S2 centered. OPEN width ≈ {open_w} steps (dir {approach_dir}).")
#         except RuntimeError as e:
#             print(f"[Rehome] S2 centering failed: {e}")
#             return
#     else:
#         # --- Normal approach: optionally fast pre-roll, then index/center ---
#         steps_far = abs(step_count)
#         if steps_far > FAR_STEP_THRESHOLD:
#             fast_steps = steps_far - FAR_STEP_THRESHOLD
#             move_biased(fast_steps, FAST_STEP_DELAY, approach_dir)

#         try:
#             open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=True)
#             print(f"S2 centered. OPEN width ≈ {open_w} steps (dir {approach_dir}).")
#         except RuntimeError as e:
#             print(f"[Homing] S2 centering failed: {e}")
#             return

#     # Apply your calibrated small offset (directional) from S2 center to true home
#     off = int(HOME_OFFSET_DIR.get(approach_dir, 0))
#     if off:
#         move_biased(off, CENTER_STEP_DELAY, approach_dir)

#     # Light settle
#     #move_biased(3, CENTER_STEP_DELAY, approach_dir)
#     #move_biased(3, CENTER_STEP_DELAY, -approach_dir)

#     # Ensure both sensors OPEN with bounded micro-tweak
#     if _micro_tweak_around_center(approach_dir):
#         print("Arrived at home (both sensors OPEN).")
#     else:
#         print("At reference, but both sensors not OPEN. Increase TWEAK_RANGE or adjust HOME_OFFSET_DIR.")

#     # Zero counters and persist now that we have a fresh, indexed home
#     _zero_s1_counters()
#     step_count = 0
#     try:
#         _persist_step_count()
#     except NameError:
#         print("Step count persistence not implemented. Check File issue.")  # during early testing
#         pass


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
    global step_count

    # Choose approach based on your convention; keep this mapping consistent
    if step_count > 0:
        approach_dir = -1   # e.g., if +step_count means you're CW of home, approach CCW
    elif step_count < 0:
        approach_dir = +1
    else:
        approach_dir = +1   # default when exactly at home

    s2_is_open = not _read_blocked(optical_pin2)
    bump_dir = -approach_dir  # leave the OPEN window opposite to the approach

    print(f"[go_home2] step_count={step_count}, s2_open={s2_is_open}, approach_dir={approach_dir}, bump_dir={bump_dir}")

    try:
        if s2_is_open:
            # --- ALREADY IN OPEN: do the simple “100-step then normal homing” ---
            move_biased(REHOME_BUMP_STEPS, EDGE_STEP_DELAY, bump_dir)
            _step_until_state(optical_pin2, True, bump_dir, EDGE_STEP_DELAY)  # ensure BLOCKED

            # Now do a single centering pass in the *approach_dir* (slow)
            open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=False)
            print(f"[Rehome@Open] S2 centered. OPEN≈{open_w} steps (dir {approach_dir}).")

        else:
            # --- NOT IN OPEN: optional fast pre-roll TOWARD home in the SAME sign as approach_dir ---
            steps_far = abs(step_count)
            if steps_far > FAR_STEP_THRESHOLD:
                fast_steps = steps_far - FAR_STEP_THRESHOLD
                move_biased(fast_steps, FAST_STEP_DELAY, approach_dir)  # <— use approach_dir here, not a separate fast_chunk_dir

            # Center in approach_dir (fast first pass OK)
            open_w = _center_on_open_window(optical_pin2, approach_dir, first_fast=True)
            print(f"[Rehome] S2 centered. OPEN≈{open_w} steps (dir {approach_dir}).")

    except RuntimeError as e:
        # One safe retry in the opposite direction
        alt_dir = -approach_dir
        print(f"[Rehome] Centering failed in dir {approach_dir}: {e}. Retrying in dir {alt_dir}.")
        open_w = _center_on_open_window(optical_pin2, alt_dir, first_fast=False)
        print(f"[Rehome] Retry succeeded. OPEN≈{open_w} steps (dir {alt_dir}).")
        approach_dir = alt_dir  # downstream offset/tweak should follow the dir actually used

    # Apply per-direction offset from S2 center to true Disk Home
    off = int(HOME_OFFSET_DIR.get(approach_dir, 0))
    if off:
        move_biased(off, CENTER_STEP_DELAY, approach_dir)

    # Final bounded tweak so both sensors are OPEN
    if _micro_tweak_around_center(approach_dir):
        print("Arrived at Disk Home (both sensors OPEN).")
    else:
        print("At reference, but both sensors not OPEN. Adjust TWEAK_RANGE or HOME_OFFSET_DIR.")

    _zero_s1_counters()  # (doesn't touch step_count)
    # If you want step_count to represent “steps from Disk Home”, keep this:
    step_count = 0
    _persist_step_count()



def go_optical_home():
    """
    Go to Disk Home (repeatable, via sensors), then apply the saved optical-home offset.
    """
    global OPTICAL_HOME_OFFSET_STEPS
    go_home2()  # zeros step_count at Disk Home
    if OPTICAL_HOME_OFFSET_STEPS != 0:
        _stage_move_signed(OPTICAL_HOME_OFFSET_STEPS)
    print(f"[OpticalHome] At optical home (offset {OPTICAL_HOME_OFFSET_STEPS} steps from disk home).")

def find_optical_home_offset():
    """
    Guide: go to Disk Home, then step the stage until the user says they're at optical home.
    Controls:
      ↑  : +1 step (CW)
      ↓  : -1 step (CCW)
      PgUp / PgDn : ±10 steps
      r  : reset this session's offset to 0 (return to Disk Home)
      Enter or s : save offset and exit
      q or Esc   : cancel without saving
    """
    global OPTICAL_HOME_OFFSET_STEPS

    # 1) Start from a clean Disk Home
    go_home2()

    # Live session offset we’re editing (don’t overwrite saved value until user saves)
    session_offset = 0
    running = True

    print("\n--- Set Optical Home Offset ---")
    print("Use ↑/↓ to nudge (PgUp/PgDn = ±10). Press Enter or 's' to save, 'r' to reset, 'q'/Esc to cancel.")

    # We use pynput's keyboard.Listener (already imported as 'keyboard' in your file)
    def _nudge(n):
        nonlocal session_offset
        if n == 0:
            return
        direction = 1 if n > 0 else -1
        move_biased(abs(n), TWEAK_STEP_DELAY, direction)
        session_offset += n
        print(f"[Offset] session = {session_offset} steps (saved = {OPTICAL_HOME_OFFSET_STEPS})")

    def on_press(key):
        nonlocal running, session_offset
        try:
            if key == keyboard.Key.up:
                _nudge(+1)
            elif key == keyboard.Key.down:
                _nudge(-1)
            elif key == keyboard.Key.page_up:
                _nudge(+10)
            elif key == keyboard.Key.page_down:
                _nudge(-10)
            elif key == keyboard.Key.enter or (hasattr(key, "char") and key.char and key.char.lower() == 's'):
                # Save & exit
                OPTICAL_HOME_OFFSET_STEPS = session_offset
                _persist_optical_offset()
                print(f"[Saved] Optical-home offset set to {OPTICAL_HOME_OFFSET_STEPS} steps.")
                running = False
                return False
            elif key == keyboard.Key.esc or (hasattr(key, "char") and key.char and key.char.lower() == 'q'):
                print("[Cancel] Leaving offset unchanged.")
                running = False
                return False
            elif hasattr(key, "char") and key.char and key.char.lower() == 'r':
                # Return to Disk Home within this session
                if session_offset != 0:
                    _nudge(-session_offset)  # back to zero
                    print("[Reset] Session offset reset to 0 (Disk Home).")
        except Exception as e:
            print(f"[Listener] {e}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while running:
            monitor_sensors()
            time.sleep(0.05)
    finally:
        listener.stop()
    print("Done setting optical-home offset.")



def go_home():
    global step_count, rev_count, steps_till_rev, steps_since_rev, transition_sequence, step_delay
    global home_revs, home_steps, home_pre_rev_steps, home_delta_steps
    print("\nMoving to Home Position...")

    # Calculate total steps needed to get back to home
    # Determine direction: -1 = reverse, 1 = forward
    rev_diff = rev_count 
    step_diff = step_count

    #total steps taking during go_home process
    homing_steps = 0

    #print(f"Current revs: {rev_count}, home revs: {home_revs}")
    #print(f"Current steps: {step_count}, home steps: {home_steps}")
    #print(f"Revolution difference: {rev_diff}, Step difference: {step_diff}")
    logging.info(f"Current revs: {rev_count}, home revs: {home_revs}")
    logging.info(f"Current steps: {step_count}, home steps: {home_steps}")
    logging.info(f"Revolution difference: {rev_diff}, Step difference: {step_diff}")
    # Calculate steps to move before reaching home

    direction = -1 if rev_diff > 0 or (rev_diff == 0 and step_diff > 0) else 1

    #move the post-revs steps 
    #move_stepper(seq, steps_since_rev, step_delay, direction)
    move_biased(steps = 1, step_delay=step_delay, direction=direction)

    homing_steps = homing_steps+ steps_since_rev
    #print(f"Moving {abs(steps_since_rev)} steps in direction {direction}...")
    logging.info(f"Moving {abs(steps_since_rev)} steps in direction {direction}...")

    current_rev = rev_count
    
    # Move full revolutions first
    if rev_diff != 0:
        #print(f"Moving {abs(rev_diff)} full revs in direction {direction}...")
        logging.info(f"Moving {abs(rev_diff)} full revs in direction {direction}...")
        while True:
            if rev_count != (current_rev + (rev_diff * direction)):
                #move_stepper(seq, 1, step_delay, direction)
                move_biased(steps =1, step_delay=step_delay, direction=direction)
                homing_steps += 1
            else: 
                break
    #print(f"Total revolutions moved: {abs(rev_diff)}")
    logging.info(f"Total revolutions moved: {abs(rev_diff)}")


    #move the pre-revs steps + hysteresis 
    slower_delay = 0.005  # Adjust for slower speed of hysteresis

    overshoot_steps = 20
    #move_stepper(seq, steps_till_rev+overshoot_steps, slower_delay, direction)
    move_biased(steps = steps_till_rev+overshoot_steps, step_delay=slower_delay, direction=direction)
    homing_steps = homing_steps + steps_till_rev + overshoot_steps
    print(f"Moving {abs(steps_till_rev + overshoot_steps)} steps in direction {direction} at slower speed...")
    
    # Move back beyond home by 60 steps in opposite direction, slower
    opposite_direction = -1* direction
    
    reverse_overshoot_steps = 10

    print(f"Reversing direction by {overshoot_steps} steps (60 steps total) at slower speed.")
    #move_stepper(seq, overshoot_steps + reverse_overshoot_steps, slower_delay, opposite_direction)
    move_biased(overshoot_steps + reverse_overshoot_steps, step_delay=slower_delay, direction=opposite_direction)
    homing_steps = homing_steps - (reverse_overshoot_steps)

    slowest_delay = slower_delay * 2

    #Slowly approach home position by moving forward 10 steps
    fine_tune_steps = 10
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
    global home_delta_steps, home_revs, home_steps, home_pre_rev_steps, step_count, rev_count, delta_steps, home, steps_till_rev, last_switch1_state, last_switch2_state , transition_sequence
    print("Please use jog mode to set worm gear to home (switch 2 open) quit jog mode.")
    time.sleep(3)
    jog_mode2(step_delay=0.001)

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
    rev_count = 0   # Reset revolution count
    steps_since_rev = 0
    steps_till_rev = 0

    # persist current position to disk
    _persist_step_count()
    
    print('Total Steps, total revs, delta steps pre-rev, delta steps post-revs:', home_steps, ' ,' ,home_revs, ' ,',home_pre_rev_steps, ' ,',home_delta_steps )

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
        print("Q) Back to main menu")

        choice = input("Select: ").strip().lower()

        if choice == "1":
            dirch = input("Approach S2 edge in direction [+/-] (ENTER=+): ").strip()
            approach_dir = -1 if dirch == "-" else +1
            _run_cancellable(calibrate_s2_steps_per_rev, approach_dir=approach_dir)  # optional

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
            _run_cancellable(move_to_wavelength, lam, m, mode=mode)
            ok, err = check_step_integrity()
            print(f"Integrity: {'OK' if ok else 'DRIFT'} (Δ={err} steps)  | step_count={step_count}")

        elif choice == "4":
            _ensure_steps_per_deg()
            print(f"S2_STEPS_PER_REV={S2_STEPS_PER_REV}, STEPS_PER_DEG={STEPS_PER_DEG:.6f}")
            ok, err = check_step_integrity()
            print(f"Integrity: {'OK' if ok else 'DRIFT'} (Δ={err} steps)  | step_count={step_count}")

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
