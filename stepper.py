#pip install pyfirmata keyboard

from pyfirmata import Arduino
import time
import keyboard

board = Arduino('COM10')  # Set your Arduino port here

pins = [11, 10, 9, 8]  # IN1–IN4 pins on ULN2003

# Define your step sequence (half-step example)
seq = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1]
]

step_delay = 0.001  # 01ms (adjust for your setup)
steps_per_move = 512  # number of steps per move (adjust as needed)

def move_stepper(sequence, steps):
    for _ in range(steps):
        for step in sequence:
            for pin, val in zip(pins, step):
                board.digital[pin].write(val)
            time.sleep(step_delay)
    # Turn all pins off when done
    for pin in pins:
        board.digital[pin].write(0)

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
            if ms < 2 or ms > 1000:
                print("Enter a value between 2 and 1000 ms.")
                continue
            new_delay = ms / 1000.0
            print(f"Speed set to {ms} ms per step.")
            return new_delay
        except ValueError:
            print("Invalid input. Enter a number.")

def jog_mode(step_delay):
    print("\n--- Jog Mode ---")
    print("Hold UP for forward, DOWN for reverse. Press Q to quit jog mode.")
    try:
        while True:
            if keyboard.is_pressed('q'):
                print("Exiting jog mode.\n")
                break
            elif keyboard.is_pressed('up'):
                # Move one step forward
                for step in seq:
                    for pin, val in zip(pins, step):
                        board.digital[pin].write(val)
                    time.sleep(step_delay)
            elif keyboard.is_pressed('down'):
                # Move one step in reverse
                for step in reversed(seq):
                    for pin, val in zip(pins, step):
                        board.digital[pin].write(val)
                    time.sleep(step_delay)
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

def main():
    global step_delay
    print("Stepper Motor Control")
    print("F: Forward")
    print("R: Reverse")
    print("J: Jog Mode (UP/DOWN arrows)")
    print("S: Speed Settings")
    print("Q: Quit")
    while True:
        cmd = input("Enter option Forward, Reverse, Jog, Speed Settings, Quit (F/R/J/S/Q): ").strip().upper()
        if cmd == 'F' or cmd == 'f':
            print(f"Moving forward at {int(step_delay * 1000)} ms/step...")
            move_stepper(seq, steps_per_move, step_delay)
        elif cmd == 'R' or cmd == 'r':
            print(f"Moving reverse at {int(step_delay * 1000)} ms/step...")
            move_stepper(list(reversed(seq)), steps_per_move, step_delay)
        elif cmd == 'J' or cmd == 'j':
            jog_mode(step_delay)
        elif cmd == 'S' or cmd == 's':
            step_delay = set_speed(step_delay)
        elif cmd == 'Q' or cmd == 'q':
            print("Quitting.")
            break
        else:
            print("Invalid option. Enter F, R, J, S, or Q.")
    for pin in pins:
        board.digital[pin].write(0)
    board.exit()

if __name__ == '__main__':
    main()
