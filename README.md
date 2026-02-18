# Monochromator Control

Python-based control and calibration tooling for a stepper-driven monochromator using an Arduino and optical switches. The main interface is `controller.py`, an interactive CLI that supports homing, calibration, and positioning by steps, angle, or wavelength (Littrow model with fitted references).

## Features
- Interactive menu for GoTo, Jog, Speed, Home, Calibration, and Position.
- Optical switch feedback for the small disk (S1) and worm gear (S2).
- Homing and optical-home offset.
- Calibration for steps/rev, steps/deg, backlash, and directional bias.
- Wavelength calibration with stored references and linear fit.
- Persistent state in `state/` and logs in `logs/`.

## Hardware
- Arduino running StandardFirmata.
- Stepper motor + ULN2003 driver (IN1-IN4 on Arduino pins 11, 10, 9, 8).
- Optical switches on Arduino pins 7 (S1) and 5 (S2).
- Appropriate motor power supply.

Wiring and system documentation:
- `Monochromator Documentation.pdf`
- `VM-504 SD3 (V2.A) (1).pdf`
- `complete system wiring.png`
- `Stepper Controller Wiring Diagram.png`
- `opticalswitch_diagram.png`
- `Dual Switch wiring.png`

## Software
- Python 3.9+ (tested on 3.13)
- `pyfirmata`, `pyserial`, `pynput`
- Optional: `keyboard` (some Windows setups)

Install:
```bash
python -m pip install pyfirmata pyserial pynput
# optional (Windows)
python -m pip install keyboard
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
4. Run the controller:
```bash
python controller.py
```

## Using the CLI
Main menu options:
- `G` GoTo relative steps (+CW / -CCW)
- `J` Jog mode (UP/DOWN arrows)
- `S` Speed settings
- `H` Home menu
- `C` Calibration
- `P` Position menu (angle/wavelength)
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

