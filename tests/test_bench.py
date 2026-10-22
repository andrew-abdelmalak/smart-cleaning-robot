"""Tests for the site's simulated hardware, site/bench.py.

The site runs the real firmware with this fake hardware in the browser.
These tests load it under CPython and check that the firmware reacts
as it does in the other tests: it cleans, avoids, recovers from a stall,
and stops when the phone disconnects.
"""

import importlib.util
import json
import shutil
import subprocess
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType

import pytest

BENCH_PATH = Path(__file__).parent.parent / "site" / "bench.py"
FAKED = ("time", "machine", "bluetooth", "ubinascii", "micropython", "main")


@pytest.fixture
def stand() -> Iterator[ModuleType]:
    """A freshly loaded bench; the real modules come back afterwards."""
    saved = {name: sys.modules.get(name) for name in FAKED}
    sys.modules.pop("main", None)
    try:
        spec = importlib.util.spec_from_file_location("bench", BENCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        for name, original in saved.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


def start_cleaning(stand: ModuleType) -> None:
    stand.bench.send("CLEAN")
    stand.bench.step(1000)


def test_clean_spins_both_wheels_forward(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.step(1000)

    fw = stand.fw
    assert fw.autonomous_state == "FWD"
    assert fw.fan.value() == 1
    assert stand.bench.forward == [True, True]
    assert min(fw.left_pulses, fw.right_pulses) > 30
    # The PID keeps the two wheels within a few pulses of each other.
    assert abs(fw.left_pulses - fw.right_pulses) <= 3


def test_object_within_15_cm_starts_a_left_pivot(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.set_distance(10)
    stand.bench.step(200)

    assert stand.fw.autonomous_state == "AVOID"
    assert stand.bench.forward == [False, True]


def test_pivot_lasts_turn_pulses(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.set_distance(10)
    stand.bench.step(200)
    stand.bench.set_distance(80)

    fw = stand.fw
    turned = 0
    for _ in range(200):
        turned = (fw.left_pulses - fw.pivot_start_left) + (
            fw.right_pulses - fw.pivot_start_right
        )
        stand.bench.step(20)
        if fw.autonomous_state != "AVOID":
            break

    assert fw.autonomous_state in ("START_FWD", "FWD")
    assert turned >= fw.TURN_PULSES - 1


def test_held_wheel_triggers_stall_recovery(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.hold_wheel(0, True)
    stand.bench.step(1500)

    assert stand.fw.autonomous_state == "STALL_TURN"


def test_button_press_toggles_cleaning(stand: ModuleType) -> None:
    stand.bench.press_button()
    stand.bench.step(800)

    assert stand.fw.autonomous_mode is True


def test_manual_forward_blocks_and_honks_near_object(stand: ModuleType) -> None:
    stand.bench.set_distance(10)
    stand.bench.send("FORWARD")
    stand.bench.step(100)

    assert stand.fw.forward_blocked is True
    assert stand.fw.horn_active is True
    assert stand.bench.speed == [0.0, 0.0]


def test_disconnect_stops_everything(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.disconnect()
    stand.bench.step(500)

    assert stand.fw.autonomous_mode is False
    assert stand.fw.ena.duty_u16() == stand.fw.enb.duty_u16() == 0


def test_state_is_json(stand: ModuleType) -> None:
    state = json.loads(stand.bench.state())

    assert state["name"] == "PicoRobot-3C33"
    assert state["connected"] is True
    # Every field site/bench.js reads.
    assert {
        "name", "connected", "pending", "state", "manual", "autonomous",
        "forward", "speed", "blocked", "horn", "duty", "fullStop", "fan",
        "led", "ledPattern", "pulses", "turns",
    } <= state.keys()  # fmt: skip


def test_obstacle_threshold_is_below_15_cm(stand: ModuleType) -> None:
    start_cleaning(stand)
    stand.bench.set_distance(15)
    stand.bench.step(200)
    assert stand.fw.autonomous_state == "FWD"

    stand.bench.set_distance(14)
    stand.bench.step(200)
    assert stand.fw.autonomous_state == "AVOID"


def test_wheel_speed_does_not_depend_on_sonar_time(stand: ModuleType) -> None:
    # The wheels keep turning while the firmware waits for the echo, so a
    # far wall (long echo) or none (30 ms timeout) must not slow the robot.
    speeds = []
    for cm in (40, None):
        stand.bench.set_distance(cm)
        if not stand.fw.autonomous_mode:
            start_cleaning(stand)
        stand.bench.step(1000)
        before = stand.bench.turns[0]
        stand.bench.step(1000)
        speeds.append(stand.bench.turns[0] - before)
    assert speeds[1] == pytest.approx(speeds[0], rel=0.05)


def test_wheel_model_inverts_the_firmware_feed_forward(stand: ModuleType) -> None:
    # bench.py copies kS and kV from ff_pwm; driving at the firmware's duty
    # for a speed must give that speed back, within one duty step (1 / kV).
    fw = stand.fw
    for pps in (20, 50, 75):
        fw.set_motor_pwm(
            fw.ff_pwm(pps, "L"), fw.ff_pwm(pps * fw.RIGHT_TRIM, "R"), "forward"
        )
        assert stand.bench._target_pps(0) == pytest.approx(pps, abs=1.6)
        assert stand.bench._target_pps(1) == pytest.approx(pps, abs=1.7 / fw.RIGHT_TRIM)


def test_bench_compiles_for_micropython(tmp_path: Path) -> None:
    # The browser runs bench.py on MicroPython, not CPython.
    mpy_cross = shutil.which("mpy-cross")
    if mpy_cross is None:
        pytest.skip("mpy-cross is not installed")
    out = tmp_path / "bench.mpy"
    subprocess.run([mpy_cross, "-o", str(out), str(BENCH_PATH)], check=True)
    assert out.stat().st_size > 0
