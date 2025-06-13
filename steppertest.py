from pyfirmata import Arduino, util
import time

#if error with pyfirmata library, open the pyfirmata.py file in the library and replace getargspec with getfullargspec function calls

board = Arduino('COM10')  # Change to your actual port

# Define pins
IN1 = 11
IN2 = 10
IN3 = 9
IN4 = 8

pins = [IN1, IN2, IN3, IN4]

for pin in pins:
    board.digital[pin].mode = 1  # OUTPUT

seq_og = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1]
]

seq = [
    [1,0,0,0],
    [0,1,0,0],
    [0,0,1,0],
    [0,0,0,1]
]

for i in range(512):
    for step in seq:
        for p, v in zip(pins, step):
            board.digital[p].write(v)
        time.sleep(0.001)

for p in pins:
    board.digital[p].write(0)


board.exit()
