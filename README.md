# Monochromator Control

Python-based control and calibration tooling for a stepper-driven monochromator using an Arduino and optical switches. The main interface is a Pyside6 python gui alongside `controller.py`, an interactive CLI that supports homing, calibration, and positioning by steps, angle, or wavelength (Littrow model with fitted references).

<img width="2846" height="1780" alt="image" src="https://github.com/user-attachments/assets/8ba8ba2d-5124-4048-ab57-3d34771cc65e" />


 
## Features
- Interactive menu for GoTo, Jog, Speed, Home, Calibration, and Position.
- Optical switch feedback for the small disk (S1) and worm gear (S2).
- Homing and optical-home offset.
- Calibration for steps/rev, steps/deg, backlash, and directional bias.
- Wavelength calibration with stored references and linear fit.
- Scanning function between wavelengths or grating angles. 
- Persistent state in `state/` and logs in `logs/`.

## Hardware
- Arduino running StandardFirmata.
- Stepper motor + ULN2003 driver (IN1-IN4 on Arduino pins 11, 10, 9, 8).
- Optical switches on Arduino pins 7 (S1) and 5 (S2).
- Appropriate motor power supply.

Wiring and system documentation:
- `Monochromator Documentation.pdf`
- `GUI_Documentation_Addendum.md`
- `VM-504 SD3 (V2.A) (1).pdf`
- `complete system wiring.png`
- `Stepper Controller Wiring Diagram.png`
- `opticalswitch_diagram.png`
- `Dual Switch wiring.png`

The GUI addendum contains the newer desktop GUI installation notes, ZWO camera setup, common GUI errors, and a feature-by-feature guide for the control, calibration, scan, and camera panes.



## Software
Python:
- Python 3.9+ (tested on 3.13)

Core controller packages:
- `pyfirmata`
- `pyserial`
- `pynput`

GUI packages:
- `PySide6`

Optional packages:
- `keyboard` for some Windows keyboard-hook setups
- `numpy` for ZWO live analysis features such as line profiles, centroid, peak finding, and ROI-assisted analysis
- `zwoasi` for the ZWO camera pane

macOS / ZWO camera dependencies:
- `libusb` installed through Homebrew
- ZWO ASI SDK `libASICamera2.dylib`
- On Apple Silicon Macs, use the `mac_arm64` SDK library, not the older Intel-only `mac` build

Recommended GUI install:
```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pyfirmata pyserial pynput PySide6
python -m pip install numpy
python -m pip install zwoasi
```

CLI-only install:
```bash
python -m pip install pyfirmata pyserial pynput
# optional (Windows)
python -m pip install keyboard
```

macOS ZWO camera install notes:
```bash
brew install libusb
```

If the ZWO SDK is not auto-detected, set the SDK path in the GUI or export it before launching:
```bash
export ZWO_ASI_LIB="/full/path/to/libASICamera2.dylib"
```

Apple Silicon example:
```bash
export ZWO_ASI_LIB="$HOME/Downloads/ASI_Camera_SDK/ASI_linux_mac_SDK_V1.41/lib/mac_arm64/libASICamera2.dylib"
```

## Quick Start
1. Upload StandardFirmata to the Arduino.
2. Connect the Arduino and verify the serial port.
3. Optional: set `ARDUINO_PORT` to override auto-detect:
```bash
# macOS
export ARDUINO_PORT=/dev/cu.usbmodem1101
# Windows (opens new terminal afterward)
setx ARDUINO_PORT COM5
```
4. Run the desktop GUI:
```bash
python monochromator_gui.py
```
Refresh Ports -> Select Arduino Port -> Initalize Controller ->ptionally connect to ZWO Camera. 

You can also autostart the gui via 
```bash
Launch Monochromator GUI.command   // Mac
Launch Monochromator GUI.bat       // windows
Launch Monochromator GUI.sh        // Linux
```
5. Or run the original CLI controller:
```bash
python controller.py
```

## Using the CLI
Main menu options:
- `G` GoTo relative steps (+CW / -CCW)
- `J` Jog mode (UP/DOWN arrows)
- `V` Speed settings
- `H` Home menu
- `C` Calibration
- `P` Position menu (angle/wavelength)
- `S` Scan menu
- `Q` Quit

Suggested first-time flow:
1. `H` -> Reset Disk Home
2. `C` -> Small disk steps/rev
3. `C` -> Large disk steps/deg (S2)
4. `C` -> Backlash calibration (optional)
5. `P` -> Wavelength calibration (add refs, fit model)
6. `P` -> Move to angle or wavelength

## State and Logs
- `state/calibration.json` stores steps/rev, steps/deg, backlash, and grating params.
- `state/wavelength_cal.json` stores wavelength references and the fitted model.
- `state/step_count.txt` stores the last step count for restart continuity.
- `state/optical_home_offset.txt` stores the optical-home offset.
- `logs/stepper.log` stores runtime logs and auto-trims at about 10 MB.

## Troubleshooting
- If Arduino fails to initialize, set `ARDUINO_PORT` or check available ports.
- On macOS, `pynput` may require Accessibility permissions for keyboard events.
- If motion drifts, re-run calibration and verify optical switch alignment.

## Other Scripts
- `stepper.py`, `stepper_mac.py`: simpler stepper test harnesses.
- `monochromator.py`: serial control for SD3 monochromator commands.
- `carousel.py`, `laser_serial.py`, `serial_generic.py`: auxiliary serial tools.
