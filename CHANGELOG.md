# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html). The firmware's public interface is its BLE command set, its pin assignments, and its tuned constants.

## [Unreleased]

## [1.1.0] - 2026-10-20

The firmware now has host-side tests, CI, and fixes for five control-logic bugs, and the project has a site where its real firmware runs in the browser. The BLE commands, pins, and tuned constants are unchanged.

### Added

- 42 tests that run the real firmware on a computer against fake `machine`, `bluetooth`, and `time` modules, with a clock that wraps like MicroPython's (#1).
- CI that lints, checks formatting, runs the tests, and builds the firmware with `mpy-cross` on every push and pull request (#1).
- The thesis LaTeX source, with a CI build (#1).
- Contributing guide, security policy, code of conduct, issue forms, and `CITATION.cff` (#1).
- A [project site](https://andrew-abdelmalak.github.io/smart-cleaning-robot/) that runs the unchanged firmware in the browser with MicroPython WebAssembly. You drive the robot around a drawn room from an on-screen remote (#2, #3).
- A 34-second project video on the site and in the README (#3).
- "Ask the thesis" on the site: semantic search over the thesis that links each result to its page in the PDF (#4).

### Changed

- The firmware moved from `main.py` to `firmware/main.py`. Copy it to the board as `main.py`, as before (#1).
- The control loop is split into command, manual, and autonomous steps, with shared PID and feed-forward helpers, type hints, and narrower exception handling. Behavior is unchanged except for the fixes below (#1).
- The README is rewritten, with diagrams, demo clips, and results checked against the thesis. It quotes the build cost in Egyptian pounds, as the thesis does (#1, #2).

### Fixed

- A second HORN now switches the horn off. Before, it was ignored while the horn sounded (#1).
- After a phone disconnects, the robot advertises again, so it can be found without a reboot (#1).
- An emergency stop during an obstacle block now clears the block. Before, the motors kept buzzing and CLEAN, FAN, and HORN were refused until a move-away command (#1).
- A command sent just before a disconnect no longer runs when the next phone connects (#1).
- During an obstacle block, HORN and FORWARD now silence the warning buzz. FORWARD stays blocked until the robot moves away (#1).

## [1.0.0] - 2026-03-19

The firmware as submitted with the bachelor thesis (June 2025), published on GitHub.

### Added

- MicroPython firmware for the Raspberry Pi Pico W: BLE remote control, ultrasonic obstacle avoidance, wheel-encoder odometry, and feed-forward plus PID motion control.
- Thesis PDF, README, a lint workflow, and the MIT license.

[Unreleased]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/releases/tag/v1.0.0
