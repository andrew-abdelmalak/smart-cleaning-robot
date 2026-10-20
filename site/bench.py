"""Simulated hardware for the real firmware on the project site.

bench.js loads MicroPython (WebAssembly) in the browser, copies
firmware/main.py into its file system, and runs this file. Every hardware
module the firmware imports is replaced by a fake, as in tests/conftest.py;
the firmware itself runs unchanged.

Each wheel turns at the speed the firmware's own feed-forward model
predicts for its PWM duty, scaled by a floor-load factor, and every encoder
pulse goes through the firmware's interrupt handler. bench.js moves the
robot through a drawn room from the wheel turns and feeds the ultrasonic
reading and collisions back in.
"""

import binascii
import json
import sys

TICKS_PERIOD = 1 << 30  # MicroPython ticks wrap here on the rp2 port
PULSES_PER_REV = 20  # 20-slot optical encoder disc
MOTOR_LAG_MS = 60  # spin-up time constant, chosen for the animation
PHYSICS_STEP_MS = 5


class Namespace:
    """Stands in for a module: the firmware only reads attributes."""


class Clock:
    """Simulated microsecond clock; sleeps advance the motor model."""

    def __init__(self) -> None:
        # Start one second after boot: the firmware treats the robot as fully
        # stopped only once its motors have been off for 100 ms.
        self.us = 1_000_000
        self.on_sleep = None

    def ticks_us(self) -> int:
        self.us += 10  # busy-wait loops such as the sonar echo always end
        return self.us % TICKS_PERIOD

    def ticks_ms(self) -> int:
        return (self.us // 1000) % TICKS_PERIOD

    def ticks_diff(self, a: int, b: int) -> int:
        half = TICKS_PERIOD // 2
        return (a - b + half) % TICKS_PERIOD - half

    def ticks_add(self, a: int, b: int) -> int:
        return (a + b) % TICKS_PERIOD

    def sleep_us(self, us: int) -> None:
        self.us += us

    def sleep_ms(self, ms: int) -> None:
        self.us += ms * 1000
        if self.on_sleep:
            self.on_sleep(ms)


class Pin:
    """Minimal machine.Pin: a value and an interrupt handler."""

    IN, OUT, PULL_UP = 0, 1, 2
    IRQ_RISING = 1

    def __init__(self, pin_id: int, mode: int = -1, pull: int | None = None) -> None:
        self._value = 1 if pull == Pin.PULL_UP else 0
        self.handler = None

    def value(self, v: int | None = None) -> int | None:
        if v is None:
            return self._value
        self._value = 1 if v else 0

    def on(self) -> None:
        self._value = 1

    def off(self) -> None:
        self._value = 0

    def high(self) -> None:
        self._value = 1

    def low(self) -> None:
        self._value = 0

    def toggle(self) -> None:
        self._value ^= 1

    def irq(self, trigger: int | None = None, handler=None) -> None:
        self.handler = handler


class PWM:
    """Minimal machine.PWM that stores a 16-bit duty."""

    def __init__(self, pin: Pin) -> None:
        self._freq = 0
        self._duty = 0

    def freq(self, f: int | None = None) -> int | None:
        if f is None:
            return self._freq
        self._freq = f

    def duty_u16(self, d: int | None = None) -> int | None:
        if d is None:
            return self._duty
        if not 0 <= d <= 65535:
            raise ValueError("duty out of range")
        self._duty = d


class BLE:
    """Minimal bluetooth.BLE: the bench fires its IRQ events."""

    def __init__(self) -> None:
        self.handler = None
        self.rx = b""

    def active(self, state: bool) -> None:
        pass

    def irq(self, handler) -> None:
        self.handler = handler

    def gatts_register_services(self, services: tuple) -> tuple:
        return ((1, 2),)

    def gap_advertise(self, interval_us: int, adv_data: bytes | None = None) -> None:
        pass

    def gatts_read(self, handle: int) -> bytes:
        return self.rx


class Sonar:
    """HC-SR04 trigger and echo pins: the echo stays high for the round
    trip to distance_cm, or never rises when distance_cm is None."""

    def __init__(self, clock: Clock) -> None:
        self.clock = clock
        self.distance_cm = 80.0
        self._fired_at = None

    def high(self) -> None:
        pass

    def low(self) -> None:
        self._fired_at = self.clock.us

    def value(self):
        if self.distance_cm is None or self._fired_at is None:
            return 0
        start = self._fired_at + 100
        width = self.distance_cm * 2 / 0.0343
        return 1 if start <= self.clock.us < start + width else 0


clock = Clock()

_time = Namespace()
for _name in (
    "ticks_us",
    "ticks_ms",
    "ticks_diff",
    "ticks_add",
    "sleep_us",
    "sleep_ms",
):
    setattr(_time, _name, getattr(clock, _name))

_machine = Namespace()
_machine.Pin = Pin
_machine.PWM = PWM
_machine.unique_id = lambda: b"\xe6\x61\x3c\x33"

_bluetooth = Namespace()
_bluetooth.UUID = str
_bluetooth.FLAG_NOTIFY = 0x10
_bluetooth.FLAG_WRITE = 0x08
_bluetooth.BLE = BLE

_ubinascii = Namespace()
_ubinascii.hexlify = binascii.hexlify

sys.modules["time"] = _time
sys.modules["machine"] = _machine
sys.modules["bluetooth"] = _bluetooth
sys.modules["ubinascii"] = _ubinascii
try:
    import micropython  # noqa: F401  (built in on MicroPython)
except ImportError:
    _micropython = Namespace()
    _micropython.const = lambda x: x
    sys.modules["micropython"] = _micropython

import main as fw  # noqa: E402  (the fakes must be in place first)

sonar = Sonar(clock)
fw.trig = fw.echo = sonar


class Bench:
    """The hardware around the firmware: phone link, wheels, the lid
    button, and the distance the ultrasonic sensor sees."""

    def __init__(self) -> None:
        self.radio = BLE()
        self.uart = fw.BLEUART(self.radio)
        self.speed = [0.0, 0.0]  # left, right, in pulses per second
        self.forward = [True, True]
        self.turns = [0.0, 0.0]  # wheel revolutions, signed
        self.partial = [0.0, 0.0]
        self.held = [False, False]
        self.load = 1.0  # floor speed as a fraction of free-wheel speed
        self.target_us = clock.us
        self.button_release_us = None
        clock.on_sleep = self.run_motors
        self.connect()

    # Phone link and commands -------------------------------------------

    def connect(self) -> None:
        self.radio.handler(1, (1, 0, b""))

    def disconnect(self) -> None:
        self.radio.handler(2, (1, 0, b""))

    def send(self, command: str) -> None:
        if fw.ble_connected:
            self.radio.rx = command.encode()
            self.radio.handler(3, (1, 2))

    def press_button(self) -> None:
        fw.button.value(0)
        self.button_release_us = clock.us + 150_000

    def hold_wheel(self, index: int, held: bool) -> None:
        self.held[index] = bool(held)

    def set_distance(self, cm: float | None) -> None:
        sonar.distance_cm = None if cm is None else float(cm)

    def set_load(self, fraction: float) -> None:
        self.load = float(fraction)

    # Motors --------------------------------------------------------------

    def _target_pps(self, index: int) -> float:
        # set_motor_pwm sends the left command to ENB and the right to ENA.
        if index == 0:
            duty = fw.enb.duty_u16() * 100 / 65535
            ks, kv, trim = 27, 0.66, 1
        else:
            duty = fw.ena.duty_u16() * 100 / 65535
            ks, kv, trim = 30, 0.61, fw.RIGHT_TRIM
        # Invert the firmware's feed-forward fit, duty = kS + kV * pps. The
        # right wheel is assumed to be as fast as RIGHT_TRIM compensates for.
        return max(0.0, (duty - ks) / kv / trim)

    def run_motors(self, ms: int) -> None:
        if self.button_release_us is not None and clock.us >= self.button_release_us:
            fw.button.value(1)
            self.button_release_us = None
        self.forward = [fw.in3.value() == 1, fw.in1.value() == 1]
        pins = (fw.left_enc, fw.right_enc)
        steps = max(1, ms // PHYSICS_STEP_MS)
        dt = ms / steps
        for _ in range(steps):
            for i in (0, 1):
                if self.held[i]:
                    self.speed[i] = 0.0
                    continue
                target = self._target_pps(i) * self.load
                self.speed[i] += (target - self.speed[i]) * min(1.0, dt / MOTOR_LAG_MS)
                pulses = self.speed[i] * dt / 1000
                sign = 1 if self.forward[i] else -1
                self.turns[i] += sign * pulses / PULSES_PER_REV
                self.partial[i] += pulses
                while self.partial[i] >= 1:
                    self.partial[i] -= 1
                    pins[i].handler(pins[i])  # single channel: always counts up

    # Loop ----------------------------------------------------------------

    def step(self, ms: int) -> None:
        """Run the firmware's control loop for ms more of simulated time.

        A loop pass takes 20 ms plus the sonar echo time, so a single call
        can overshoot; the target carries over so the simulated clock keeps
        pace with real time.
        """
        self.target_us += ms * 1000
        while clock.us < self.target_us:
            fw.control_step()

    def state(self) -> str:
        """Return the firmware and stand state as JSON for bench.js."""
        return json.dumps(
            {
                "t": clock.us // 1000,
                "name": fw._ADVERTISING_NAME,
                "connected": fw.ble_connected,
                "autonomous": fw.autonomous_mode,
                "state": fw.autonomous_state,
                "manual": fw.manual_command or "",
                "blocked": fw.forward_blocked,
                "horn": fw.horn_active,
                "fullStop": fw.is_full_stop(),
                "led": fw.led.value(),
                "ledPattern": fw.led_state,
                "fan": fw.fan.value(),
                "pulses": [fw.left_pulses, fw.right_pulses],
                "duty": [
                    round(fw.enb.duty_u16() * 100 / 65535),
                    round(fw.ena.duty_u16() * 100 / 65535),
                ],
                "speed": [round(self.speed[0], 1), round(self.speed[1], 1)],
                "forward": self.forward,
                "turns": [round(self.turns[0], 3), round(self.turns[1], 3)],
                "held": self.held,
                "distance": sonar.distance_cm,
            }
        )


bench = Bench()
