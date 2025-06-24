from pyfirmata import Arduino, util
import time
import keyboard

board = Arduino('COM10')  # Change to your actual port
optical_pin = 7

# Start the iterator to receive input from the board
it = util.Iterator(board)
it.start()

# Set up pin 7 as input
board.digital[optical_pin].mode = 0  # 0 means INPUT

print("Reading optical switch on D7. Press Ctrl+C to exit.")
try:
    while True:
        value = board.digital[optical_pin].read()
        if value is None:
            print("Waiting for pin state...")
        elif value == 0:
            print("SWITCH OPEN (slit present in sensor)")
        else:
            print("SWITCH BLOCKED (beam blocked by metal disk)")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nExiting.")
    board.exit()
