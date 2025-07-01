#pyserial

import serial
import io
import time
import sys
import datetime

class LaserIO(object):


    def __init__(self, laser_port = "/dev/ttyS6"):


        self.laser_port = laser_port
        self.ser = serial.Serial(self.laser_port,9600,timeout = 0.05)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1),  
                               newline = '\r',
                               line_buffering = True)
        self.normal_mode()
    

    def normal_mode(self):
        print("normal mode")
        self.ser_io.write("R0\r")
        out = self.ser_io.readline()
        print(out)

    def measure(self,verbose = True):
        '''
        Get laser head measurement through serial
        '''
        self.ser_io.write('M0\r')
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
        #print(head_val)

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

class Carousel(object):

    def __init__(self, port = "/dev/ttyUSB0"):
        self.port= port
        self.ser = serial.Serial(self.port,57600,timeout = 0.05, xonxoff = True)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser),  
                               newline = '\r\n',
                               line_buffering = True)
    def home(self):
        self.ser_io.write('1OR\r\n')
        out = self.ser_io.readline()
        state =  self.status()
	
        #self.ser_io.write('1pa0\r\n')
	#out = self.ser_io.readline()
        #state = self.status()	

        timeout = time.time() + 60*2   #  2 minutes from now
        while state[7:9] != '32':
            
            
            if state[3:7] != '0000':
                print(state)
                print(state[3:7])
                self.status()
                print('controller error')
                break

            elif state[7:9] == '32':
                print('32: Ready from homing')
                break
            elif state[7:9] == '33':
                print('33: Ready from moving')
                break
            elif state[7:9] == '1E':
                print('1E: Homing from RS232')
                
            elif state[7:9] == '28':
                print('28: Moving')
            elif state[7:9] == '0A':
                print('0A: Not Referenced from reset')
                self.ser_io.write('1OR\r\n')
                out = self.ser_io.readline()
                state =  self.status()
            elif state[7:9] == '0B':
                print('0B: Not Referenced from homing')
                self.ser_io.write('1OR\r\n')
                out = self.ser_io.readline()
                state =  self.status()
            else:
                print("unexpected code")
                state = self.status()
                




            if time.time() > timeout:
                print('homing timeout')
                break
            time.sleep(1)
            state = self.status()



    def pa(self,angle):
        command = '1PA' +  str(angle) + '\r\n'
        self.ser_io.write(command)
        out = self.ser_io.readline()
        print(out)
        return out
    def com(self,command):
        self.ser_io.write(command)
        out = self.ser_io.readline()
        print(out)
        return out
        

    """timeout = time.time() + 60*5 # 5 minutes from now
        while state[7:9] != '33':
            time.sleep(1)
            
            if state[3:7] != '0000':
                print('controller error')
                break

            
            if state[7:9] == '33':
                print('Ready from moving')
                break

            if time.time() > timeout:
                print('moving timeout')
                break
            state = self.status()"""

    def status(self,verbose = True):
        command = '1TS' + '\r\n'
        self.ser_io.write(command)
        out = self.ser_io.readline()
        if (verbose):
            print(out)
        return out

    def pos(self,verbose = True):
        #reads position from controller, returns float of position
        
        command = '1TP' + '\r\n'
        self.ser_io.write(command)
        out = self.ser_io.readline()
        position = float(out[3:-2])
        if (verbose):
            print(out)
        return position

class Metrology(object):
    def __init__(self, carousel_port = "/dev/ttyUSB0", laser_port = "/dev/ttyS6", file_prefix =  'metrology'):


        self.laser_port = laser_port
        self.carousel_port = carousel_port
        self.laser = LaserIO(laser_port)
        self.carousel = Carousel(carousel_port)

        self.laser.normal_mode()
        self.laser.measure()
        self.carousel.home()

        print('Serials ready!')

    def sweep(self, start, end, tag = ''):

        position = self.carousel.pos()
        if position != start:
            self.carousel.pa(start)
            timeout = time.time() + 60*5 # 5 minutes from now
            state = self.carousel.status()
            while state[7:9] != '33':
                time.sleep(1)
                
                if state[3:7] != '0000':
                    print('controller error')
                    break
                elif state[7:9] == '28':
                    position = self.carousel.pos()  
                    print('28: Moving... Position = ' + str(position))
                elif state[7:9] == '33':
                    print('Ready from moving')
                    break
                else:
                    print('unexpected code')
                    state = self.carousel.status()
                if time.time() > timeout:
                    print('moving timeout')
                    break
                state = self.carousel.status()
        print('At start position')
        print('Begin sweep')
        start_time = time.time()

        self.carousel.pa(end)
        logfile  =  open('metrology_'+ tag + '_' +  str(start) +'_'+ str(end) + '_' + str(datetime.date.today())+ '_' + str(str(datetime.datetime.now().time())) + ".txt" , 'w')
        state = self.carousel.status()
        timeout = time.time() + 60*10
        while state[7:9] != '33':
                
                zlaser = self.laser.measure(verbose = False)
                position = self.carousel.pos(verbose = False)

                line = str(position) + ("\t")+ str(zlaser) + ("\t") + str(time.time() - start_time) + ("\n") 
                logfile.write(line)
                print(line)
                
                if state[3:7] != '0000':
                    print('controller error')
                    break

                
                if state[7:9] == '33':
                    print('Ready from moving')
                    break

                if time.time() > timeout:
                    break
                time.sleep(0.1)
                state = self.carousel.status()
        logfile.close()









#def main():
print("loaded")

"""duration = sys.argv[1]
    tag =  sys.argv[2]
    laser = LaserIO("/dev/cu.usbserial-110")    logfile  =  open("laser_log_"+ str(tag)+ '_' + str(datetime.date.today())+ '_' + str(time.time()) + ".txt" , 'w')
    start_time  = time.time()
    while(time.time() - start_time < int(duration)):
        
        zlaser = laser.measure()
        line = str(time.time() - start_time) + ("\t") + str(zlaser) + ("\n")
        logfile.write(line)
    
    logfile.close()"""

#if __name__ == "__main__":
#    main()
        
