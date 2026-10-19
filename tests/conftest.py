"""Host-side fakes for the MicroPython modules the firmware imports.

The firmware touches hardware at import time, so each test imports a fresh
copy of ``main`` with fake ``machine``, ``bluetooth``, ``micropython``,
``ubinascii``, and ``time`` modules in place. A simulated clock drives every
``ticks_*`` and ``sleep_*`` call, so loop tests run instantly and in order.
"""

import binascii
import importlib
import sys
import time as real_time
import types
from collections.abc import Callable, Iterator

import pytest


class LoopExit(Exception):
    """Raised by the fake clock to stop ``main_program_loop`` after N sleeps."""


class FakeClock:
    """Microsecond clock that advances on every read and every sleep.

    Each ``ticks_us`` read advances 10 us, so busy-wait loops such as the
    ultrasonic echo wait always terminate.
    """

    def __init__(self) -> None:
        self.us = 0
        self.on_sleep: Callable[[int], None] | None = None

    def ticks_us(self) -> int:
        self.us += 10
        return self.us

    def ticks_ms(self) -> int:
        return self.us // 1000

    @staticmethod
    def ticks_diff(a: int, b: int) -> int:
        return a - b

    @staticmethod
    def ticks_add(a: int, b: int) -> int:
        return a + b

    def sleep_us(self, us: int) -> None:
        self.us += us

    def sleep_ms(self, ms: int) -> None:
        self.us += ms * 1000
        if self.on_sleep is not None:
            self.on_sleep(ms)


class FakePin:
    """Minimal ``machine.Pin``: holds a value and an IRQ handler."""

    IN, OUT, PULL_UP = 0, 1, 2
    IRQ_RISING = 1

    def __init__(self, pin_id: int, mode: int = -1, pull: int | None = None) -> None:
        self.id = pin_id
        self._value = 1 if pull == FakePin.PULL_UP else 0
        self.handler: Callable[["FakePin"], None] | None = None

    def value(self, v: int | None = None) -> int | None:
        if v is None:
            return self._value
        self._value = int(bool(v))
        return None

    def on(self) -> None:
        self._value = 1

    def off(self) -> None:
        self._value = 0

    high, low = on, off

    def toggle(self) -> None:
        self._value ^= 1

    def irq(self, trigger: int, handler: Callable[["FakePin"], None]) -> None:
        self.handler = handler


class FakePWM:
    """Minimal ``machine.PWM`` that records frequency and 16-bit duty."""

    def __init__(self, pin: FakePin) -> None:
        self.pin = pin
        self._freq = 0
        self._duty = 0

    def freq(self, f: int | None = None) -> int | None:
        if f is None:
            return self._freq
        self._freq = f
        return None

    def duty_u16(self, d: int | None = None) -> int | None:
        if d is None:
            return self._duty
        if not 0 <= d <= 65535:
            raise ValueError(f"duty out of range: {d}")
        self._duty = d
        return None


class FakeBLE:
    """Minimal ``bluetooth.BLE`` that lets tests fire IRQ events."""

    instances: list["FakeBLE"] = []

    def __init__(self) -> None:
        self.handler: Callable[[int, tuple], None] | None = None
        self.rx = b""
        self.advertise_calls = 0
        FakeBLE.instances.append(self)

    def active(self, state: bool) -> None:
        pass

    def irq(self, handler: Callable[[int, tuple], None]) -> None:
        self.handler = handler

    def gatts_register_services(self, services: tuple) -> tuple:
        return ((1, 2),)

    def gap_advertise(self, interval_us: int, adv_data: bytes) -> None:
        self.adv_data = adv_data
        self.advertise_calls += 1

    def gatts_read(self, handle: int) -> bytes:
        return self.rx


class FakeSonar:
    """HC-SR04 pair: the echo pin goes high for the round-trip time of
    ``distance_cm`` after the trigger pin falls."""

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.distance_cm: float | None = None  # None means no echo at all
        self._fired_at: int | None = None

    # Trigger pin interface
    def high(self) -> None:
        pass

    def low(self) -> None:
        self._fired_at = self.clock.us

    # Echo pin interface
    def value(self) -> int:
        if self.distance_cm is None or self._fired_at is None:
            return 0
        start = self._fired_at + 100
        width = self.distance_cm * 2 / 0.0343
        return 1 if start <= self.clock.us < start + width else 0


def _fake_modules(clock: FakeClock) -> dict[str, types.ModuleType]:
    machine = types.ModuleType("machine")
    machine.Pin = FakePin
    machine.PWM = FakePWM
    machine.unique_id = lambda: b"\xe6\x61\x3c\x33"

    bluetooth = types.ModuleType("bluetooth")
    bluetooth.UUID = str
    bluetooth.FLAG_NOTIFY = 0x10
    bluetooth.FLAG_WRITE = 0x08
    bluetooth.BLE = FakeBLE

    micropython = types.ModuleType("micropython")
    micropython.const = lambda x: x

    ubinascii = types.ModuleType("ubinascii")
    ubinascii.hexlify = binascii.hexlify

    fake_time = types.ModuleType("time")
    for name in ("ticks_us", "ticks_ms", "ticks_diff", "ticks_add"):
        setattr(fake_time, name, getattr(clock, name))
    fake_time.sleep_us = clock.sleep_us
    fake_time.sleep_ms = clock.sleep_ms

    return {
        "machine": machine,
        "bluetooth": bluetooth,
        "micropython": micropython,
        "ubinascii": ubinascii,
        "time": fake_time,
    }


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def robot(clock: FakeClock, monkeypatch: pytest.MonkeyPatch) -> Iterator:
    """A freshly imported firmware module wired to fakes and a sonar."""
    FakeBLE.instances.clear()
    for name, module in _fake_modules(clock).items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("main", None)
    module = importlib.import_module("main")
    # Hand the real time module back to pytest; the firmware keeps its fake.
    monkeypatch.setitem(sys.modules, "time", real_time)
    sonar = FakeSonar(clock)
    module.trig = sonar
    module.echo = sonar
    module.sonar = sonar
    module.clock = clock
    yield module
    sys.modules.pop("main", None)


@pytest.fixture
def run_loop(robot, clock: FakeClock) -> Callable[..., list[str]]:
    """Run ``main_program_loop`` for a number of sleeps.

    ``commands`` maps a sleep index to a BLE command string that arrives
    just before the next loop iteration, or to a callable to run at that
    point (for example, pressing the button). When ``wheels_turn`` is true,
    each encoder gains one pulse per sleep while its motor has non-zero duty.
    """

    def run(
        sleeps: int,
        commands: dict[int, str | Callable[[], None]] | None = None,
        wheels_turn: bool = True,
    ) -> list[str]:
        """Return the autonomous state observed at each sleep."""
        commands = commands or {}
        states: list[str] = []
        count = 0

        def on_sleep(ms: int) -> None:
            nonlocal count
            count += 1
            states.append(robot.autonomous_state)
            if wheels_turn:
                # set_motor_pwm routes the left command to ENB and the
                # right command to ENA, to match the physical wiring.
                if robot.enb.duty_u16() > 0:
                    robot.left_encoder_isr(None)
                if robot.ena.duty_u16() > 0:
                    robot.right_encoder_isr(None)
            event = commands.get(count)
            if callable(event):
                event()
            elif event:
                robot.ble_connected = True
                robot.last_ble_command = event
            if count >= sleeps:
                raise LoopExit

        clock.on_sleep = on_sleep
        try:
            robot.main_program_loop()
        except LoopExit:
            pass
        finally:
            clock.on_sleep = None
        return states

    return run
