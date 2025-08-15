#pip install pyfirmata keyboard

import os
import logging
from datetime import datetime

from pyfirmata import Arduino, util 
import time
import keyboard
import sys
import termios
from pynput import keyboard


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
revs_per_rotation = 205 
delta_steps = 40

#use these variable to set a home. 
home_steps = 0
home_revs = 0
home_delta_steps =0 
home_pre_rev_steps =0 
#boolean to track home
home = False

# --- Homing config (tune as needed) ---
HOME_OFFSET_DIR = {+1: 15, -1: 15}  # steps from S2 center toward the approach_dir to exact home
FAST_STEP_DELAY   = 0.001
EDGE_STEP_DELAY   = 0.004
CENTER_STEP_DELAY = 0.006
TWEAK_STEP_DELAY  = 0.008
MAX_FIND_STEPS    = 20000       # hard cap so loops can't run forever
TWEAK_RANGE       = 40           # +/- small steps to satisfy "both open"
FAR_STEP_THRESHOLD = 200   # steps to consider "far" for fast pre-roll


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

def move_biased(steps, step_delay, direction):
    # Apply direction-change backlash and steady-state reverse scaling.
    global _last_move_dir, _rev_scale_err_accum

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


def move_stepper(seq, steps,step_delay, direction):
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home, switch1, switch2, steps_per_rev, steps_till_rev, steps_since_rev, is_1rev_completed
    # do NOT reinitialize them here!
    # just update their values as you go

    if direction == -1:
        sequence = list(reversed(seq))
    else: sequence = seq
    

    for _ in range(steps):
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

        if is_1rev_completed == False and state1 == "BLOCKED":
            steps_till_rev += 1 *direction

    # Turn all pins off when done
    for pin in pins:
        board.digital[pin].write(0)

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
    print("Hold UP for forward CCW, DOWN for reverse CW. Press Q to quit jog mode.")
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

    #orginal running loop, testing other loops for responsiveness
    # while running:
    #     if forward_pressed:
    #         move_stepper(seq, steps =1, step_delay =0.001, direction= 1)
    #         #for testing    
    #         print("step count", step_count)
    #         print("rev count", rev_count)
    #     if reverse_pressed:
    #         move_stepper(seq, steps=1, step_delay =0.001, direction= -1)
    #         #for testing    
    #         print("step count", step_count)
    #         print("rev count", rev_count)
    #     monitor_sensors()
    #     time.sleep(0.001)  # Adjust for responsiveness/smoothness

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

def _center_on_open_window(pin, approach_dir):
    """
    Centers the sensor on the OPEN window using the chosen approach_dir.
    We land at the far (OPEN->BLOCKED) edge and reverse by half width.
    Backlash is handled by move_biased().
    """
    w = _find_B2O_and_measure_open(pin, direction=approach_dir, first_fast=True)
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


#edge + step based homing function, as opposed to the only step based homing function gohome() 
def go_home2():
    """
    Shortest-path homing:
      1) Choose approach_dir from signed step_count (short way back).
      2) On switch 2, approach BLOCKED->OPEN edge in that direction,
         measure OPEN width, reverse half to center (backlash compensated).
      3) Apply per-direction home offset to reproduce your calibrated 'both-open' home.
      4) Micro-tweak a few steps to satisfy both sensors OPEN (bounded, no runaway).
    """
    global step_count

    # 0) Pick shortest approach: +steps => go CW (-1), -steps => go CCW (+1)
    approach_dir = -1 if step_count >= 0 else +1

    # 1) Fast pre-roll if far away (uses counters only to save time; sensor-indexing comes next)
    steps_far = abs(step_count)
    if steps_far > FAR_STEP_THRESHOLD:
        fast_steps = steps_far - FAR_STEP_THRESHOLD
        move_biased(fast_steps, FAST_STEP_DELAY, approach_dir)

    # 2) Index & center on S2 OPEN window (direction-aware; uses sensors only)
    try:
        open_w = _center_on_open_window(optical_pin2, approach_dir)
        print(f"S2 centered. OPEN width ≈ {open_w} steps (dir {approach_dir}).")
    except RuntimeError as e:
        print(f"[Homing] S2 centering failed: {e}")
        return

    # 3) Apply small per-direction offset from S2 center to your true home
    off = int(HOME_OFFSET_DIR.get(approach_dir, 0))
    if off:
        move_biased(off, CENTER_STEP_DELAY, approach_dir)

    # 4) Optional light hysteresis settle: overshoot a tiny bit and return
    # (kept small so it never runs long)
    move_biased(3, CENTER_STEP_DELAY, approach_dir)
    move_biased(3, CENTER_STEP_DELAY, -approach_dir)

    # Uncomment if you want to bias using a static offset number of steps that are known
    # if approach_dir == 1:
    #     move_biased(8, EDGE_STEP_DELAY, 1)
    # else:
    #     move_biased(2, EDGE_STEP_DELAY, -1)

    # 5) Ensure both sensors OPEN (bounded micro-tweak, no infinite while)
    if _micro_tweak_around_center(approach_dir):
        print("Arrived at home (both sensors OPEN).")
    else:
        print("At reference, but both sensors not OPEN. Increase TWEAK_RANGE or adjust HOME_OFFSET_DIR.")



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
    
    print('Total Steps, total revs, delta steps pre-rev, delta steps post-revs:', home_steps, ' ,' ,home_revs, ' ,',home_pre_rev_steps, ' ,',home_delta_steps )

def home_menu():
    while True:
        print("\n--- Home Menu ---")
        print("1: Go to Home")
        print("2: Reset Home")
        print("Q: Return to main menu")
        choice = input("Choose an option (1/2/Q): ").strip().lower()

        if choice == '1':
            print("Moving to home position...")
            #go_home()
            go_home2()
            break
        elif choice == '2':
            confirm = input("Are you sure you want to reset home? This will re-set the monochromator. (y/N): ").strip().lower()
            if confirm == 'y':
                set_home()
                break
            else:
                print("Reset cancelled. Returning to home menu.")
        elif choice == 'q':
            print("Exiting home menu.")
            break
        else:
            print("Invalid option. Please enter 1, 2, or Q.")


def wait_for_initial_sensor_states():
    global switch1, switch2
    while switch1 is None or switch2 is None:
        switch1 = board.digital[optical_pin].read()
        switch2 = board.digital[optical_pin2].read()
        time.sleep(0.01)  # Wait briefly

def setup_logging():
    # Create logs directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Create timestamp for filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"logs/stepper_log_{timestamp}.txt"
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filename),
            logging.StreamHandler()
        ]
    )
    return log_filename


def main():
    global step_delay

    wait_for_initial_sensor_states    

    # Initialize logging
    log_file = setup_logging()
    logging.info(f"Starting stepper motor control - logging to {log_file}")

    print("Stepper Motor Control")
    print("F: Forward CCW")
    print("R: Reverse CW")
    print("J: Jog Mode (UP/DOWN arrows)")
    print("S: Speed Settings")
    print("H: Home Menu")
    print("C: Calibration")
    print("Q: Quit")

    while True:
        cmd = input("Enter option Forward, Reverse, Jog, Speed Settings, Home, Calibration, Quit (F/R/J/S/H/C/Q): ").strip().upper()
        if cmd in ('F','f'):
            print(f"Moving forward at {int(step_delay * 1000)} ms/step...")
            #move_stepper(seq, steps_per_move, step_delay, direction= 1)
            move_biased(steps=1, step_delay=step_delay, direction=1)
            flush_input()
        elif cmd in ('R', 'r'):
            print(f"Moving reverse at {int(step_delay * 1000)} ms/step...")
            #move_stepper(seq, steps_per_move, step_delay, direction= -1)
            move_biased(steps=1, step_delay=step_delay, direction=-1)
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

        elif cmd in ('q', 'Q'):
            print("Quitting.")
            flush_input()
            break
        else:
            print("Invalid option. Enter F, R, J, S, H, C, or Q.")
            flush_input()
    for pin in pins:
        board.digital[pin].write(0)
    board.exit()

if __name__ == '__main__':
    main()
