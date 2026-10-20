"""Tests for the website's test stand, site/bench.py.

The stand runs the real firmware with fake hardware in the browser. These
tests load it under CPython and check that the firmware reacts on the stand
as it does in the other tests: it cleans, avoids, recovers from a stall,
and stops when the phone disconnects.
"""

import importlib.util
import json
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
    spec = importlib.util.spec_from_file_location("bench", BENCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    yield module
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
