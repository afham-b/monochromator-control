#pyserial

import serial
import io
import time
import sys

import datetime

class LaserIO(object):


    def __init__(self, laser_port = "/dev/ttyS6"):


        self.laser_port = laser_port
        self.ser = serial.Serial(self.laser_port,9600,timeout = 0.25)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1),  
                               newline = '\r',
                               line_buffering = True)
        self.normal_mode()
    

    def normal_mode(self):
        print("normal mode")
        self.ser_io.write(u"R0\r")
        out = self.ser_io.readline()
        print(out)

    def measure(self,verbose = True):
        '''
        Get laser head measurement through serial
        '''
        self.ser_io.write(u'M0\r')
        out = self.ser_io.readline()
        if verbose: 
            print(out)

        i = out.rfind("M0,") + 3
        if i == -1:
            print("M0 measurement not found")
            return None
        j = out[i:].find(",")
        #print(i)
        head_val =  out[i:i+j]
        print(head_val)

        if self.isfloat(head_val):
            distance  = float(head_val)
            return distance 
        else:
            print("Laser head out of range")
            return None

    def isfloat(self,value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def close(self):
        self.ser.close()
        return None


def main():
    print("measuring")
    duration = sys.argv[1]
    #tag =  sys.argv[2]
    laser = LaserIO()
    logfile  =  open("laser_log_"+ '_' + str(datetime.date.today())+ '_' + str(time.time()) + ".txt" , 'w')
    start_time  = time.time()
    while(time.time() - start_time < int(duration)):
        
        zlaser = laser.measure()
        line = str(time.time() - start_time) + ("\t") + str(zlaser) + ("\n")
        logfile.write(line)
    
    logfile.close()

sif __name__ == "__main__":
    main()
        
