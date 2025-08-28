from pyfirmata import Arduino, util
import time

#if error with pyfirmata library, open the pyfirmata.py file in the library and replace getargspec with getfullargspec function calls

#board = Arduino('COM10')  # Change to your actual port

try:
    board = Arduino('/dev/cu.usbmodem1101')
except Exception as e:
    print(f"Arduino initialization failed: {e}")
    sys.exit(1)

# Define pins
IN1 = 11
IN2 = 10
IN3 = 9
IN4 = 8

pins = [IN1, IN2, IN3, IN4]

for pin in pins:
    board.digital[pin].mode = 1  # OUTPUT

seq1 = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1]
]

seq2 = [
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1],
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1],
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]
]

seq3 = [
    [1,0,0,0],   # Step 1
    [1,1,0,0],   # Step 2
    [0,1,0,0],   # Step 3
    [0,1,1,0],   # Step 4
    [0,0,1,0],   # Step 5
    [0,0,1,1],   # Step 6
    [0,0,0,1],   # Step 7
    [1,0,0,1]    # Step 8
]


for i in range(512):
    for step in seq1:
        for p, v in zip(pins, step):
            board.digital[p].write(v)
        time.sleep(0.001)

for p in pins:
    board.digital[p].write(0)


board.exit()