# Smart Cleaning Robot

A MicroPython-based autonomous and manual smart cleaning robot controller for Raspberry Pi Pico W.

The project combines:
- BLE command and telemetry channel (UART-style GATT service)
- Differential drive motor control with encoder feedback
- PID-based straight and pivot motion correction
- Obstacle detection with ultrasonic sensor
- Safety handling (stall checks, emergency stop, brake behavior)
- Runtime status feedback through LED and peripheral outputs

## Features

- BLE advertising with unique per-device name (`PicoRobot-XXXX`)
- Manual command mode and autonomous state-machine mode
- Encoder interrupt counting for closed-loop movement correction
- Feed-forward + PID wheel balancing
- Obstacle-aware navigation thresholds
- Stall watchdog and timed safety checks

## Hardware Targets

- Raspberry Pi Pico W (MicroPython firmware)
- Dual DC motor driver (PWM + direction pins)
- Left/right wheel encoders
- Ultrasonic distance sensor (TRIG/ECHO)
- Fan output and status LED
- User button input

## Repository Structure

- `main.py` : Robot firmware entry point and control logic
- `README.md` : Project overview and setup
- `LICENSE` : MIT license
- `.gitignore` : Python/editor/OS ignore rules

## Pin Mapping (from source)

- Motor PWM: `ENA=6`, `ENB=7`
- Motor direction: `IN1=12`, `IN2=13`, `IN3=14`, `IN4=15`
- Encoders: `LEFT=21`, `RIGHT=20`
- Peripherals: `FAN=8`, `LED=18`, `TRIG=10`, `ECHO=11`, `BUTTON=16`

## Getting Started

1. Install MicroPython on Pico W.
2. Copy `main.py` to the board filesystem.
3. Power the robot and connect over BLE to the advertised `PicoRobot-XXXX` device.
4. Send movement/autonomous control commands via the BLE UART service UUID:
   - Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
   - TX Char (notify): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
   - RX Char (write): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`

## Safety Notes

- Test with wheels off-ground before floor operation.
- Verify motor wiring polarity before autonomous mode.
- Tune PID gains and threshold constants to your mechanical build.

## Author

Andrew Khalil Samuel Abdelmalak

**Supervisors:** Prof. Dr. Walid Atef Hafez Omran | Co-Supervisor: Dr. Hisham Mostafa El Sherif

## Thesis

The full bachelor thesis is available in [`docs/Thesis.pdf`](docs/Thesis.pdf).

## License

MIT
