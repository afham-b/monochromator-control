#pip install pyfirmata keyboard

from pyfirmata import Arduino, util 
import time
import keyboard
import sys
import termios
from pynput import keyboard


#windows
#board = Arduino('COM10')  # Set your Arduino port here

#mac, open terminal and type this for com number ls /dev/tty.*
board = Arduino('/dev/cu.usbmodem1101')

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
                        rev_count += 1*direction
                        is_1rev_completed = True
                        steps_since_rev = 0
                        #print(f"Full revolution detected! Total revolutions: {rev_count}")
                        transition_sequence = ["OPEN"]  # reset for next revolution

            last_switch1_state = state1
            time.sleep(step_delay)

        step_count += 1*direction  # After one complete seq, count as one step
        steps_since_rev += 1*direction
        if is_1rev_completed == False:
            if state1 == "BLOCKED":
                steps_till_rev += 1*direction

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
                move_stepper(seq, steps=1, step_delay =0.001, direction= 1)
                 #for testing    
                print("step count", step_count)
                print("rev count", rev_count)

            elif keyboard.is_pressed('down'):
                # Move one step in reverse
                move_stepper(seq, steps =1, step_delay =0.001, direction= -1)
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

def calibration():
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home, steps_per_rev, revs_per_rotation, steps_since_rev
    print("\n--- Calibration ---")
    print("Press 1 to calibrate the SMALL disk (optical switch 1).")
    print("Press 2 to calibrate the LARGE worm gear disk (optical switch 2).")
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
        if forward_pressed:
            move_stepper(seq, steps =1, step_delay =0.001, direction= 1)
            #for testing    
            print("step count", step_count)
            print("rev count", rev_count)
        if reverse_pressed:
            move_stepper(seq, steps=1, step_delay =0.001, direction= -1)
            #for testing    
            print("step count", step_count)
            print("rev count", rev_count)
        monitor_sensors()
        time.sleep(0.001)  # Adjust for responsiveness/smoothness

    listener.stop()

def monitor_sensors():
    global step_count, rev_count, switch1, switch2
    
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
 
    print(f"Sensor 1: {state1} | Sensor 2: {state2}")

    if state2 == "OPEN":
        print("Home position detected! Step count reset to 0.")
        step_count = 0
        rev_count = 0
        home = True

    #only print if a state has changed 
    # if (state1, state2) != tuple(last_states):
    #     print(f"Sensor 1: {state1} | Sensor 2: {state2}")
    #     last_states[0], last_states[1] = state1, state2  # update in-place

def go_home():
    global step_count, rev_count, steps_till_rev, steps_since_rev
    global home_revs, home_steps, home_pre_rev_steps, home_delta_steps
    print("\nMoving to Home Position...")

    # Calculate total steps needed to get back to home
    # Determine direction: -1 = reverse, 1 = forward
    rev_diff = rev_count - home_revs
    step_diff = step_count - home_steps

    print(f"Current revs: {rev_count}, home revs: {home_revs}")
    print(f"Current steps: {step_count}, home steps: {home_steps}")

    direction = -1 if rev_diff > 0 or (rev_diff == 0 and step_diff > 0) else 1

    # Move full revolutions first
    if rev_diff != 0:
        print(f"Moving {abs(rev_diff)} full revs in direction {direction}...")
        for _ in range(abs(rev_diff)):
            move_stepper(seq, steps_per_rev, step_delay, direction)
    
    # Move remaining steps if not aligned
    remaining_steps = step_count - home_steps
    if remaining_steps != 0:
        print(f"Moving {abs(remaining_steps)} steps in direction {direction}...")
        move_stepper(seq, abs(remaining_steps), step_delay, direction)

    print("Arrived at home position!")


def set_home():
    global home_delta_steps, home_revs, home_steps, home_pre_rev_steps, step_count, rev_count, delta_steps, home, steps_till_rev, last_switch1_state, last_switch2_state , transition_sequence
    print("Please use jog mode to set worm gear to home (switch 2 open) quit jog mode, then run home setting steps as needed.")
    time.sleep(3)
    jog_mode2(step_delay=0.001)

    init_steps = step_count
    init_rev_count = rev_count
    init_delta_steps = delta_steps

    print("Now that the large Worm Gear is set to open, move the motor using jog untill optical home is acheived, then quit jog mode")
    time.sleep(4)
    jog_mode2(step_delay=0.001)
    
    home_delta_steps = steps_since_rev
    home_revs = rev_count
    home_steps = step_count
    home_pre_rev_steps = steps_till_rev
    
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
            go_home()
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


def main():
    global step_delay
    print("Stepper Motor Control")
    print("F: Forward CCW")
    print("R: Reverse CW")
    print("J: Jog Mode (UP/DOWN arrows)")
    print("S: Speed Settings")
    print("H: Return to Home")
    print("C: Calibration")
    print("Q: Quit")

    while True:
        cmd = input("Enter option Forward, Reverse, Jog, Speed Settings, Home, Calibration, Quit (F/R/J/S/H/C/Q): ").strip().upper()
        if cmd == 'F' or cmd == 'f':
            print(f"Moving forward at {int(step_delay * 1000)} ms/step...")
            move_stepper(seq, steps_per_move, step_delay, direction= 1)
            flush_input()
        elif cmd == 'R' or cmd == 'r':
            print(f"Moving reverse at {int(step_delay * 1000)} ms/step...")
            move_stepper(seq, steps_per_move, step_delay, direction= -1)
            flush_input()
        elif cmd == 'J' or cmd == 'j':
            #jog_mode(step_delay)
            # for mac, jog2 function uses pynput instead of keyboard package
            jog_mode2(step_delay)
            flush_input()
        elif cmd == 'S' or cmd == 's':
            step_delay = set_speed(step_delay)
            flush_input()

        elif cmd == 'H' or cmd == 'h': 
            home_menu()
        elif cmd == 'C' or cmd == 'c':
            print("Starting Calibration")
            calibration()

        elif cmd == 'Q' or cmd == 'q':
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
