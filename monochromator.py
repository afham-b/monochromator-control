# Monochromator control
'''
Commands:
Wavelength movement: GOTO, NM, ?NM, >NM, MONO-?DONE, MONO-STOP, NM/MIN, ?NM/MIN
Grating: GRATING, ?GRATING, ?GRATINGS, INSTALL, SELECT-GRATING, G/MM, BLAZE, UNINSTALL
Diverter: EXIT-MIRROR, ENT-MIRROR, FRONT, SIDE, ?MIRROR, ?MIR
Calibration: INIT-OFFSET, INIT-GADJUST, MONO-EESTATUS, RESTORE FACTORY SETTINGS, MONO-RESET, HELLO, MODEL, SERIAL
'''

import serial
import io
import datetime
import sys


class MonoIO(object):

    def __init__(self, dev_port="COM3"):

        self.dev_port = dev_port
        self.ser = serial.Serial(self.dev_port,9600,timeout = 2)

        self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1),
                               newline = '\r\n',
                               line_buffering = True)

    def write(self, command, verbose=True):
        '''send command and read output'''
        self.ser.write(command.encode())
        out = self.ser.read(75)
        if verbose:
            print(out)
        return out

    def listen(self, time):
        '''listen for a set amount of seconds'''
        endTime = datetime.datetime.now() + datetime.timedelta(minutes=0.3)  # 0.5?

        while True:
            if datetime.datetime.now() >= endTime:
                break
            print("Reading serial:")
            out = self.ser.readline()
            print(out)

    # WAVELENGTH: GOTO, NM, >NM, ?NM, MONO-?DONE, MONO-STOP, NM/MIN, ?NM/MIN
    def goto(self, wavelength): # device.write(b"###.#### GOTO\r\n")
        '''goes to destination wavelength'''
        goto = str(wavelength)+" GOTO\r\n"  # is wavelength already a string?
        device.write(goto)
        device.listen(5)

    def nm(self, wavelength): # Goes to destination wavelength at constant nm/min rate specified by last NM/MIN
        nm = str(wavelength)+" NM\r\n"
        device.write(nm)
        device.listen(5)

    def usernm(self, wavelength):
        '''Similar to NM except returns control to user immediately instead of waiting for completion of monochromator
        move. Can be used with ?NM or MONO-?DONE. Command must be terminated with MONO-STOP.
        Use the NM command when communication with the monochromator during scan is not required.'''
        unm = str(wavelength)+" >NM\r\n"
        device.write(unm)
        device.listen(5)

    def checknm(self):  # device.write(b"?NM\r\n")
        # checks present wavelength
        device.write("?NM\r\n")
        device.listen(5)

    def monodone(self):  # Use with >NM to determine if monochromator reached destination (1 if complete, 0 if not)
        device.write("MONO-?DONE\r\n")
        device.listen(5)

    def monostop(self):  # Stops the monochromator wavelength move after >NM
        device.write("MONO-STOP\r\n")
        device.listen(5)

    def nmmin(self, scan):  # Sets constant scan rate in nm/min to 0.01 nm/min resolution. e.g. 10.0 NM/MIN
        rate = str(scan) + " NM/MIN\r\n"
        device.write(rate)
        device.listen(5)

    def checknmmin(self):  # Returns present scan rate in nm/min to 0.01 nm/min resolution units nm/min appended
        device.write("?NM/MIN\r\n")
        device.listen(5)

    # GRATING: GRATING, ?GRATING, ?GRATINGS, INSTALL, SELECT-GRATING, G/MM, BLAZE, UNINSTALL
    def grating(self, gnum):
        # Recalls params for specified grating from non-volatile memory.
        # Up to nine (9) gratings are allowed. Grating number from 1 - 9. e.g. 3 GRATING
        grating = str(gnum) + " GRATING\r\n"
        device.write(grating)
        device.listen(5)

    def checkgrating(self):  # Returns the number of the grating presently being used numbered 1 - 9.
        device.write("?GRATING\r\n")
        device.listen(5)

    def checkgratings(self):  # Returns list of installed gratings with position, groove density, blaze.
        # The present grating is specified with an arrow.
        device.write("?GRATINGS\r\n")
        device.listen(5)

    def install(self, gnum):  # Installs new grating parameters into non-volatile memory of monochromator.
        # Uses the grating part number to specify params: 1-120-500 5 INSTALL places a 1200 g/mm grating
        # blazed at 500nm into the second grating position on #5.
        npgrating = str(gnum) + " INSTALL\r\n"
        device.write(npgrating)
        device.listen(5)

    # install grating with parameters
    def selectgrating(self,num):  # Specifies grating number to be installed 1 - 9.
        ingrating = str(num) + " SELECT-GRATING\r\n"
        device.write(ingrating)
        device.listen(5)

    def gmm(self,density):  # Specifies groove density
        groove = str(density) + " G/MM\r\n"
        device.write(groove)
        device.listen(5)

    def blaze(self): # Specifies blaze wavelength and units of grating to be installed with 7 chars of user’s choice.
        # This command is issued BEFORE the parameters. After command is issued, SD3 responds with “ “ .
        # Seven characters are then entered (numbers, letters, spaces or special characters).
        device.write("BLAZE\r\n")
        device.listen(5)

    def uninstall(self, num):  # Used to remove a grating and parameters from SD3 non-volatile memory e.g. 3 UNINSTALL
        outgrating = str(num) + " UNINSTALL\r\n"
        device.write(outgrating)
        device.listen(5)


    # DIVERTER: EXIT-MIRROR, ENT-MIRROR, FRONT, SIDE, ?MIRROR, ?MIR
    def exitmirror(self):  # Designates exit diverter mirror to receive diverter control commands. For AM
        # monochromators that can accept two diverter mirrors.
        device.write("EXIT-MIRROR\r\n")
        device.listen(5)

    def entmirror(self):  # Designates entrance diverter mirror to receive diverter control commands. For AM
        # monochromators that can accept two diverter mirrors.
        device.write("ENT-MIRROR\r\n")
        device.listen(5)

    def front(self):  # Moves designated diverter mirror to position beam to front port position.
        device.write("FRONT\r\n")
        device.listen(5)

    def side(self):  # Moves designated diverter mirror to position beam to side port position.
        device.write("SIDE\r\n")
        device.listen(5)

    def mirpos(self):  # position of designated diverter mirror: "front" or "side"
        device.write("?MIRROR\r\n")
        device.listen(5)

    def mirposnum(self):  # 0 = front, 1 = side
        device.write("?MIR\r\n")
        device.listen(5)


    # CALIRBATION: INIT-OFFSET, INIT-GADJUST, MONO-EESTATUS, RESTORE FACTORY SETTINGS, MONO-RESET, HELLO, MODEL, SERIAL

    def initoffset(self, decimal, gratnum):
        # Sets the offset value for designated grating. Default values are 25600 for all gratings.
        # NOTE: Grating designator used with this command is grating # - 1.
        # 25590. 0 INIT-OFFSET for setting offset on grating #1.
        # NOTE: Requires a decimal point after the offset value.
        # For the new parameters of this command to take effect, the monochromator must be initialized with MONO-RESET
        # or by turning power off and back on.
        gratd = int(gratnum) - 1
        offset = str(decimal) + " " + str(gratd) + "." + " INIT-OFFSET\r\n"
        device.write(offset)
        device.listen(5)

    def initgadjust(self, adjval, gratnum):
        # Sets grating adjustment value for designated grating. Default values are 10000 for all gratings.
        # The limits on parameter are + / - 1000 for all gratings.
        # Grating designator used is grating # -1
        # e.g. 9993 1 INIT-GADJUST
        # for setting gadjust on the second grating
        # "This command is to maintain compatibility with previous Acton applications. For new applications, use
        # INIT-SP300-GADJUST command below"
        # No decimal points. For the new parameters to take effect, MONO-RESET or turn power off and on
        gratn = int(gratnum) - 1
        adj = str(adjval) + "-" + str(gratn) + " INIT-GADJUST\r\n"
        device.write(adj)
        device.listen(5)

    def monoeestatus(self):  # Returns setup and grating calibration parameters for all gratings
        device.write("MONO-EESTATUS\r\n")
        device.listen(5)


    def factory(self):  # Returns all params including grating calibration to original factory calibrated settings
        # Will overwrite any calibration params set by user.
        device.write("RESTORE FACTORY SETTINGS \r\n")
        device.listen(5)

    def monoreset(self):  # initialize: need after INIT-OFFSET, INIT-GADJUST, INIT-SP300-GADJUST
        device.write("MONO-RESET\r\n")
        device.listen(5)

    def hello(self):  # same as MONO-RESET
        device.write("HELLO\r\n")
        device.listen(5)
    def serial(self):  # device.write("SERIAL\r\n")
        snum = "SERIAL\r\n"
        device.write(snum)
        device.listen(5)
    def model(self):  # device.write("MODEL\r\n")
        mnum = "MODEL\r\n"
        device.write(mnum)
        device.listen(5)


    def isfloat(self,value):
        try:
            float(value)
            return True
        except ValueError:
            return False

    def close(self):
        self.ser.close()
        return None


print("Creating serial object")

device = MonoIO()


# USER INPUT
def main():
    print("Monochromator\n")
    user = input("Enter command (M for menu):\n").upper()
    while user != "Q":
        if user == "M":
            print("COMMANDS:\n" 
                  "Wavelength movement: GOTO, NM, ?NM, >NM, MONO-?DONE, MONO-STOP, NM/MIN, ?NM/MIN \n"
                  "Grating: GRATING, ?GRATING, ?GRATINGS, INSTALL, SELECT-GRATING, G/MM, BLAZE, UNINSTALL \n"
                  "Diverter: EXIT-MIRROR, ENT-MIRROR, FRONT, SIDE, ?MIRROR, ?MIR \n"
                  "Calibration: INIT-OFFSET, INIT-GADJUST, MONO-EESTATUS, RESTORE FACTORY SETTINGS, MONO-RESET, "
                  "HELLO, MODEL, SERIAL \n"
                  "Q: quit\n")
            user = input("Enter command (M for menu):\n").upper()

        # WAVELENGTH:
        elif user == "GOTO":
            print("Enter wavelength (nm)")
            w = input("Wavelength: ")
            device.goto(w)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "NM":
            print("Enter wavelength (nm)")
            v = input("Wavelength: ")
            device.nm(v)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?NM":
            device.checknm()
            user = input("Enter command (M for menu):\n").upper()
        elif user == ">NM":
            print("Enter wavelength (nm)")
            m = input("Wavelength: ")
            device.usernm(m)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "MONO-?DONE":
            device.monodone()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "MONO-STOP":
            device.monostop()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "NM/MIN":
            print("Enter scan rate (nm/min)")
            r = input("Rate: ")
            device.nmmin(r)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?NM/MIN":
            device.checknmmin()
            user = input("Enter command (M for menu):\n").upper()

        # GRATING:
        elif user == "GRATING":
            print("Grating # (1-9):")
            g = input("Grating #: ")
            device.grating(g)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?GRATING":
            device.checkgrating()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?GRATINGS":
            device.checkgratings()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "INSTALL":
            # other params?
            x = input("Grating #: ")
            device.install(x)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "SELECT-GRATING":
            print("Grating # (1-9): ")
            gr = input("Grating #: ")
            device.selectgrating(gr)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "G/MM":
            d = input("Groove density g/mm: ")
            device.gmm(d)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "BLAZE":
            device.blaze()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "UNINSTALL":
            ug = input("Grating #: ")
            device.uninstall(ug)
            user = input("Enter command (M for menu):\n").upper()

        # DIVERTER:
        elif user == "EXIT-MIRROR":
            device.exitmirror()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "ENT-MIRROR":
            device.entmirror()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "FRONT":
            device.front()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "SIDE":
            device.side()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?MIRROR":
            device.mirpos()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "?MIR":
            device.mirposnum()
            user = input("Enter command (M for menu):\n").upper()

        # CALIBRATION:
        elif user == "INIT-OFFSET":
            gd = input("Grating (1-9): ")
            d = input("Offset value: ")
            device.initoffset(gd, d)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "INIT-GADJUST":
            gda = input("Grating (1-9): ")
            a = input("Grating adjustment value: ")
            device.initgadjust(a, gda)
            user = input("Enter command (M for menu):\n").upper()
        elif user == "MONO-EESTATUS":
            device.monoeestatus()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "RESTORE FACTORY SETTINGS":
            device.factory()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "MONO-RESET":
            device.monoreset()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "HELLO":
            device.hello()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "SERIAL":
            device.serial()
            user = input("Enter command (M for menu):\n").upper()
        elif user == "MODEL":
            device.model()
            user = input("Enter command (M for menu):\n").upper()
        else:
            print("Invalid option")
            user = input("Enter command (M for menu):\n").upper()

    if user == "Q":
        print("bye!")
        device.close()


main()





#if __name__ == "__main__":
#    main()

