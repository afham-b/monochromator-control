#pyserial

import serial
import io
import datetime
import sys

import datetime

class SerialIO(object):


    def __init__(self, dev_port = "/dev/cu.usbserial-210"):


        self.dev_port = dev_port
        self.ser = serial.Serial(self.dev_port,57600,timeout = 2)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1),  
                               newline = '\r\n',
                               line_buffering = True)
    

    '''def normal_mode(self):
        print("normal mode")
        self.ser_io.write(u"R0\r")
        out = self.ser_io.readline()
        print(out)'''

    def write(self,command,verbose = True):
        '''
        send command and read output
        '''
        self.ser_io.write(command)
        out = self.ser_io.readline()
        if verbose: 
            print(out)
        return out

    def listen(self, time):
        '''listen for a set amount of seconds'''
        endTime = datetime.datetime.now() + datetime.timedelta(minutes=1.5)

        while True:
            if datetime.datetime.now() >= endTime:
                break
            print("Reading serial:")
            out = ser.readline()
            print(out)
    


    def isfloat(self,value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def close(self):
        self.ser.close()
        return None


#def main():

print("Creating serial object")
    
device = SerialIO()

#if __name__ == "__main__":
#    main()
        
