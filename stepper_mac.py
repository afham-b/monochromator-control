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
#boolean to track home
home = False
last_switch1_state = None
last_switch2_state = None
transition_sequence = []

# determined by calibration
steps_per_rev = 0
revs_per_rotation = 0 

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
    global step_count, rev_count, last_switch1_state, last_switch2_state, transition_sequence, home
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
                        #print(f"Full revolution detected! Total revolutions: {rev_count}")
                        transition_sequence = ["OPEN"]  # reset for next revolution

            last_switch1_state = state1
            time.sleep(step_delay)

        step_count += 1*direction  # After one complete seq, count as one step
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
                # Move one step forward
                # for step in seq:
                #     for pin, val in zip(pins, step):
                #         board.digital[pin].write(val)
                #     monitor_sensors
                #     time.sleep(step_delay)
                move_stepper(seq, steps_per_move =1, step_delay =0.001, direction= 1)
                 #for testing    
                print("step count", step_count)
                print("rev count", rev_count)


            elif keyboard.is_pressed('down'):
                # Move one step in reverse
                # for step in reversed(seq):
                #     for pin, val in zip(pins, step):
                #         board.digital[pin].write(val)
                #     monitor_sensors
                #     time.sleep(step_delay)
                move_stepper(seq, steps_per_move =1, step_delay =0.001, direction= -1)
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

def step_forward(step_delay):
    for step in seq:
        for pin, val in zip(pins, step):
            board.digital[pin].write(val)
            monitor_sensors()
        time.sleep(step_delay)

def step_reverse(step_delay):
    for step in reversed(seq):
        for pin, val in zip(pins, step):
            board.digital[pin].write(val)
            monitor_sensors()
        time.sleep(step_delay)

def flush_input():
    termios.tcflush(sys.stdin, termios.TCIFLUSH)

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
        time.sleep(0.01)  # Adjust for responsiveness/smoothness

    listener.stop()

def monitor_sensors():
    global step_count, rev_count
    
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


def main():
    global step_delay
    print("Stepper Motor Control")
    print("F: Forward CCW")
    print("R: Reverse CW")
    print("J: Jog Mode (UP/DOWN arrows)")
    print("S: Speed Settings")
    print("H: Return to Home")
    print("Q: Quit")

    while True:
        cmd = input("Enter option Forward, Reverse, Jog, Speed Settings, Home, Quit (F/R/J/S/H/Q): ").strip().upper()
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
            print("Homing, please stand by")
            #go_home(step_delay)
        elif cmd == 'Q' or cmd == 'q':
            print("Quitting.")
            flush_input()
            break
        else:
            print("Invalid option. Enter F, R, J, S, H, or Q.")
            flush_input()
    for pin in pins:
        board.digital[pin].write(0)
    board.exit()

if __name__ == '__main__':
    main()
