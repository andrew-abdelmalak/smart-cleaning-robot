"""Timing logic must survive the MicroPython ticks counter wrapping.

ticks_ms() wraps every 2**30 ms (about 12.4 days). Code that subtracts ticks
directly instead of using ticks_diff() fails only across the wrap, so these
tests start the simulated clock 2 s before it.
"""

import pytest

from conftest import TICKS_PERIOD, FakeClock


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock(start_ms=TICKS_PERIOD - 2000)


@pytest.fixture
def robot(robot, clock):
    # The robot has been up for days: its motors last ran 1 s ago.
    robot.last_motor_time = clock.ticks_ms() - 1000
    return robot


def test_stall_detection_works_across_the_wrap(robot, run_loop):
    states = run_loop(120, {10: "CLEAN"}, wheels_turn=False)
    assert "STALL_TURN" in states


def test_slow_flash_works_across_the_wrap(robot, clock):
    robot.ble_connected = True
    robot.update_led()
    before = robot.led.value()
    clock.us += 3000 * 1000  # cross the wrap
    robot.update_led()
    assert robot.led.value() != before
