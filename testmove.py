from monochromator import MonoIO

mono = MonoIO(dev_port="COM12")  # or your actual port
mono.write("500.0 GOTO\r\n")  # Move to 500 nm
