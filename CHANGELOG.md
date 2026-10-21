# Changelog

All notable changes to this project are documented here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html). The firmware's public interface is its BLE command set, its pin assignments, and its tuned constants.

## [Unreleased]

### Changed

- The site's drawn room is a new 4.8 m by 2.7 m living room, chosen by running the firmware in thousands of realistic layouts. Averaged over runs with slightly different timing, it cleans about 41 % of the floor in 2 minutes and 65 % in 5, with fewer stalls and less driving over cleaned floor than before.
- The simulated ultrasonic sensor covers the HC-SR04's 30-degree measuring angle from its datasheet, not 15 degrees, so it sees walls the robot approaches at a shallow angle.
- The cleaning score counts the floor the nozzle can reach, so it can reach 100 %, and sits beside the room's buttons instead of over the drawing.

### Added

- A "4× speed" button runs the room's clock four times faster. It is off by default; the firmware and physics are the same.

## [1.1.1] - 2026-10-21

Fixes and hardening for the project site and its simulated hardware, found by code, security, and accessibility reviews. The firmware, its BLE commands, pins, and tuned constants are unchanged.

### Fixed

- On a phone, the remote kept working only while the drawn room was on screen. It now works whenever the remote is visible (#8).
- In the simulation, the wheels stood still while the firmware waited for the sonar echo, so the robot drove up to half as fast on open floor. Its pace is now the same at any distance (#8).
- Holding an arrow key toggled between driving and braking (#8).
- The log could call a command "ignored" before the firmware had read it (#8).
- The room no longer freezes silently on an error; it says so, and Reset room restarts it (#8).
- "Ask the thesis" lost keyboard focus when results re-ranked, announced download progress too often for screen readers, and could fetch its index twice (#8).
- The hero clip had no pause control, a clip paused by the visitor restarted when scrolled back into view, and the page scrolled sideways at 320 px (#8).
- The thesis search index dropped short passage tails, including the bill of materials' grand total (#8).

### Changed

- The site serves MicroPython WebAssembly, transformers.js, the ONNX runtime, and its fonts itself instead of loading them from CDNs and Google Fonts. Each library file is pinned by SHA-256 from its npm package (#8, #9).
- Dependabot proposes monthly, grouped updates for GitHub Actions, uv, and the site's browser libraries, each at least 14 days after release (#6, #7, #9).
- uv resolves only releases at least 14 days old, on a rolling window (#6).

### Security

- A Content Security Policy limits the site's scripts, styles, and fonts to its own origin; the language model's files are the only outside requests (#8).
- The thesis CI build runs LaTeX with shell escape disabled (#8).

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

[Unreleased]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.1.1...HEAD
[1.1.1]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/releases/tag/v1.0.0
