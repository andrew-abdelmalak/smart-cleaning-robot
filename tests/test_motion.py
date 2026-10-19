"""Unit tests for the motor, PID, sensor, and LED helpers in the firmware.

Expected values are worked out by hand from the gains and feed-forward
coefficients in ``firmware/main.py``, so a change to any tuned constant or
to the control math fails here first.
"""

import pytest

from conftest import duty


def pins(robot) -> tuple[int, int, int, int]:
    return (robot.in1.value(), robot.in2.value(), robot.in3.value(), robot.in4.value())


def test_advertising_name_uses_last_two_id_bytes(robot):
    assert robot._ADVERTISING_NAME == "PicoRobot-3C33"


def test_motor_pwm_runs_at_1_khz(robot):
    assert robot.ena.freq() == 1000
    assert robot.enb.freq() == 1000


@pytest.mark.parametrize(
    ("pps", "wheel", "expected"),
    [
        (75.0, "L", 76),  # 27 + 0.66 * 75
        (56.25, "R", 64),  # 30 + 0.61 * 56.25
        (0.0, "L", 27),  # static friction offset only
        (200.0, "R", 95),  # capped at 95 %
    ],
)
def test_ff_pwm(robot, pps, wheel, expected):
    assert robot.ff_pwm(pps, wheel) == expected


def test_set_motor_pwm_swaps_channels_for_wiring(robot):
    robot.set_motor_pwm(40, 20, "forward")
    assert robot.enb.duty_u16() == duty(40)  # left command
    assert robot.ena.duty_u16() == duty(20)  # right command
    assert pins(robot) == (1, 0, 1, 0)


@pytest.mark.parametrize(
    ("direction", "expected"),
    [
        ("backward", (0, 1, 0, 1)),
        ("pivot_left", (1, 0, 0, 1)),
        ("pivot_right", (0, 1, 1, 0)),
    ],
)
def test_set_motor_pwm_direction_pins(robot, direction, expected):
    robot.set_motor_pwm(30, 30, direction)
    assert pins(robot) == expected


def test_set_motor_pwm_stop_zeroes_both_channels(robot):
    robot.set_motor_pwm(50, 50, "stop")
    assert robot.ena.duty_u16() == 0
    assert robot.enb.duty_u16() == 0


def test_straight_motion_without_reference_is_feed_forward_only(robot):
    robot.straight_motion("backward")
    assert robot.enb.duty_u16() == duty(76)
    assert robot.ena.duty_u16() == duty(64)


def test_straight_motion_pid_slows_the_leading_wheel(robot):
    # Left is 10 pulses ahead: P 8.0 + I 1.5 + D 2.5 = 12 pps correction.
    robot.left_pulses = 10
    robot.straight_motion("forward", 0, 0)
    assert robot.enb.duty_u16() == duty(68)  # 75 - 12 = 63 pps
    assert robot.ena.duty_u16() == duty(71)  # 56.25 + 12 = 68.25 pps


def test_straight_motion_integral_is_clamped(robot):
    robot.left_pulses = 10
    for _ in range(3):
        robot.straight_motion("forward", 0, 0)
    assert robot.straight_integral == robot.INTEGRAL_CLAMP_LIMIT


def test_pivot_motion_progress_pid_at_start_of_turn(robot):
    # Progress error 39: P 23.4 + I 1.5 (clamped) + D 7.8 = 32.7 pps boost.
    robot.pivot_motion("left", robot.TURN_PULSES)
    assert robot.enb.duty_u16() == duty(95)  # 107.7 pps, capped
    assert robot.ena.duty_u16() == duty(79)  # 107.7 * 0.75 = 80.8 pps
    assert pins(robot) == (1, 0, 0, 1)


def test_pivot_motion_balance_term_uses_sum_of_wheel_deltas(robot):
    # Pins the hardware-tuned behavior: the balance error is -(L + R) for a
    # left pivot, not a left/right difference (see README, Known limitations).
    robot.left_pulses = 5
    robot.right_pulses = 5
    robot.pivot_motion("left", robot.TURN_PULSES)
    assert robot.balance_integral == -10
    assert robot.enb.duty_u16() == duty(80)  # 99.7 - 18.5 = 81.2 pps
    assert robot.ena.duty_u16() == duty(86)  # 74.8 + 18.5 = 93.3 pps


def test_distance_without_echo_times_out_to_999(robot):
    robot.sonar.distance_cm = None
    assert robot.get_distance_cm() == 999


def test_distance_from_echo_width(robot):
    robot.sonar.distance_cm = 20
    assert robot.get_distance_cm() == pytest.approx(20, abs=0.5)


def test_led_slow_flash_when_only_connected(robot, clock):
    robot.ble_connected = True
    robot.update_led()
    before = robot.led.value()
    clock.us += robot.LED_SLOW_FLASH_MS * 1000
    robot.update_led()
    assert robot.led_state == "SLOW_FLASH"
    assert robot.led.value() != before


def test_led_solid_in_autonomous_mode(robot):
    robot.autonomous_mode = True
    robot.update_led()
    assert robot.led_state == "SOLID_ON"
    assert robot.led.value() == 1


def test_led_rapid_flash_when_blocked(robot):
    robot.autonomous_mode = True
    robot.forward_blocked = True
    robot.update_led()
    assert robot.led_state == "RAPID_FLASH"
