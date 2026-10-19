"""Behavior tests for ``main_program_loop``: commands, safety, and the FSM.

Each test drives the real loop on simulated hardware for a fixed number of
20 ms ticks. Commands arrive at tick 10 or later, because a command is only
accepted after the motors have been idle for 100 ms.
"""

from conftest import FakeBLE

IRQ_CENTRAL_CONNECT = 1
IRQ_CENTRAL_DISCONNECT = 2
IRQ_GATTS_WRITE = 3


def duty(percent: float) -> int:
    return int(percent / 100 * 65535)


def motors_stopped(robot) -> bool:
    return robot.ena.duty_u16() == 0 and robot.enb.duty_u16() == 0


def test_forward_command_drives_forward(robot, run_loop):
    run_loop(15, {10: "FORWARD"})
    assert robot.manual_command == "FORWARD"
    assert (robot.in1.value(), robot.in3.value()) == (1, 1)
    assert not motors_stopped(robot)


def test_repeating_a_movement_command_brakes_and_stops(robot, run_loop):
    run_loop(20, {10: "FORWARD", 15: "FORWARD"})
    assert robot.manual_command is None
    assert motors_stopped(robot)


def test_obstacle_blocks_forward_and_sounds_horn(robot, run_loop):
    robot.sonar.distance_cm = 10
    run_loop(15, {10: "FORWARD"})
    assert robot.forward_blocked
    assert robot.horn_active
    assert robot.manual_command is None
    # The horn buzzes the motors at 10 % duty (7.5 % on the trimmed side).
    assert robot.enb.duty_u16() == duty(10)
    assert robot.ena.duty_u16() == duty(7.5)


def test_turning_away_clears_the_obstacle_block(robot, run_loop):
    robot.sonar.distance_cm = 10
    run_loop(18, {10: "FORWARD", 13: "LEFT"})
    assert not robot.forward_blocked
    assert not robot.horn_active
    assert robot.manual_command == "LEFT"


def test_clean_from_rest_starts_autonomous_cleaning(robot, run_loop):
    states = run_loop(14, {10: "CLEAN"})
    assert robot.autonomous_mode
    assert robot.fan.value() == 1
    assert "FWD" in states


def test_button_press_toggles_autonomous_mode(robot, run_loop):
    run_loop(
        16,
        {
            10: lambda: robot.button.value(0),
            12: lambda: robot.button.value(1),
        },
    )
    assert robot.autonomous_mode


def test_clean_while_moving_is_an_emergency_stop(robot, run_loop):
    run_loop(20, {10: "FORWARD", 15: "CLEAN"})
    assert not robot.autonomous_mode
    assert robot.manual_command is None
    assert robot.fan.value() == 0
    assert motors_stopped(robot)


def test_autonomous_mode_pivots_left_around_an_obstacle(robot, run_loop):
    robot.sonar.distance_cm = 10
    states = run_loop(60, {10: "CLEAN"})
    first_avoid = states.index("AVOID")
    assert "FWD" in states[:first_avoid]
    # After a 39-pulse pivot the robot drives forward again.
    assert "FWD" in states[first_avoid:]


def test_stalled_wheels_trigger_reverse_and_turn(robot, run_loop):
    states = run_loop(120, {10: "CLEAN"}, wheels_turn=False)
    assert "STALL_TURN" in states


def test_ble_disconnect_triggers_emergency_stop(robot, run_loop):
    run_loop(15, {10: "FORWARD"})
    ble = FakeBLE.instances[0]
    ble.handler(IRQ_CENTRAL_CONNECT, (7, 0, b""))
    ble.handler(IRQ_CENTRAL_DISCONNECT, (7, 0, b""))
    assert not robot.ble_connected
    assert robot.manual_command is None
    assert motors_stopped(robot)


def test_ble_write_is_trimmed_and_uppercased(robot):
    ble = FakeBLE()
    robot.BLEUART(ble)
    ble.rx = b" forward\r\n"
    ble.handler(IRQ_GATTS_WRITE, (7, 2))
    assert robot.last_ble_command == "FORWARD"


def test_fan_toggles_only_while_stopped(robot, run_loop):
    run_loop(25, {10: "FAN", 15: "FORWARD", 20: "FAN"})
    assert robot.fan.value() == 1


def test_horn_command_switches_the_horn_on(robot, run_loop):
    run_loop(13, {10: "HORN"})
    assert robot.horn_active
    assert robot.enb.duty_u16() == duty(10)
