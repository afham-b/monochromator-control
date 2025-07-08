from pyfirmata import Arduino, util
import time
import keyboard

#windows 
#board = Arduino('COM10')  # Change to your actual port

#mac, open terminal and type this for com number ls /dev/tty.*
board = Arduino('/dev/cu.usbmodem1101')

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

print("Reading optical switch on D7 & D5. Press Ctrl+C to exit.")
try:
    while True:
        # code for single optical switch
        # value = board.digital[optical_pin2].read()
        # if value is None:
        #    print("Waiting for pin state...")
        # elif value:
        #    print("SWITCH BLOCKED (beam blocked by disk)")
        # else:
        #    print("SWITCH Open (beam going through disk slit )")
        # time.sleep(0.1)

        value1 = board.digital[optical_pin].read()
        value2 = board.digital[optical_pin2].read()

        # Print sensor 1 state
        if value1 is None:
            print("Sensor 1: Waiting for pin state...", end=" | ")
        elif value1:
            print("Sensor 1: BLOCKED", end=" | ")
        else:
            print("Sensor 1: OPEN", end=" | ")

        # Print sensor 2 state
        if value2 is None:
            print("Sensor 2: Waiting for pin state...")
        elif value2:
            print("Sensor 2: BLOCKED")
        else:
            print("Sensor 2: OPEN")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting.")
    board.exit()
