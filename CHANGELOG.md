# Changelog

I record all notable changes to this project here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html). The firmware's public interface is its BLE command set, its pin assignments, and its tuned constants.

## [Unreleased]

## [1.3.0] - 2026-10-24

The thesis opens in a PDF viewer on the project site, the site loads the rest of the page in the background in reading order, and the site and README speak in the first person. The firmware is unchanged.

### Added

- The thesis link and each "Ask the thesis" result open the thesis in a PDF viewer on the site (Mozilla's PDF.js, pinned by version and checksum), at the right page with the passage highlighted.
- After the room's firmware starts, the site loads the rest of the page in the background, one file at a time in reading order: the videos, the PDF viewer and the thesis, then the "Ask the thesis" language model, so each part is ready when the visitor gets to it. It skips this when the browser asks to save data or is on a 2G connection; each part then loads when it is used.
- "Ask the thesis" keeps the last question and its results for the browser tab, so coming back from the PDF viewer shows them again even when the browser reloads the page.
- The project video, the only clip on the site with sound, has a "Play with sound" button above it that plays it from the start with its music. Scrolling away mutes it again.
- A test that every firmware number the site quotes still matches the firmware.

### Changed

- Each diagram on the site carries its full description as a visible caption, and the image viewer shows the whole caption on as many lines as it needs. The README describes the state machine in text below its diagram.
- The site's "For engineers" section is now "How it works", like the README's.
- The site's "4× speed" and "Sound" controls are switches.
- The README, the changelog, and the site describe my own work in the first person.
- Percentages on the site and in the README have no space before the % sign.

### Removed

- The "Inside the firmware" signal view, which was on the site for two days after #15. The room, the remote, and its readout already show the real firmware at work, and the thesis covers the rest for readers who dig deeper (#16).

### Fixed

- The image viewer opened each image where the previous one was scrolled. It now opens every image at its start.
- In the project video, the camera now moves from the remote to the robot while it honks, so the honk is on screen (#14).

## [1.2.0] - 2026-10-21

A new drawn room on the project site, a speed control, and a re-recorded project video. The firmware is unchanged.

### Changed

- The site's drawn room is a new 4.8 m by 2.7 m living room. I chose it by running the firmware in thousands of realistic layouts. Averaged over runs with slightly different timing, it cleans about 41 % of the floor in 2 minutes and 65 % in 5, with fewer stalls and less driving over cleaned floor than before.
- The simulated ultrasonic sensor covers the HC-SR04's 30-degree measuring angle from its datasheet, not 15 degrees, so it sees walls the robot approaches at a shallow angle.
- The cleaning score counts the floor the nozzle can reach, so it can reach 100 %, and sits beside the room's buttons instead of over the drawing.

### Added

- A "4× speed" button runs the room's clock four times faster. It is off by default; the firmware and physics are the same.
- The project video shows the new room, and plays muted and looping when it scrolls into view, like the site's other clips. The README shows a silent animated preview that links to the version with sound.

### Fixed

- Focus rings had less than 3:1 contrast against the page and the remote's keys (WCAG 2.2, 1.4.11). They now use a deeper tangerine on light surfaces and the lighter ember inside the remote.

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
- I rewrote the README, with diagrams and demo clips, and checked its results against the thesis. It quotes the build cost in Egyptian pounds, as the thesis does (#1, #2).

### Fixed

- A second HORN now switches the horn off. Before, it was ignored while the horn sounded (#1).
- After a phone disconnects, the robot advertises again, so it can be found without a reboot (#1).
- An emergency stop during an obstacle block now clears the block. Before, the motors kept buzzing and CLEAN, FAN, and HORN were refused until a move-away command (#1).
- A command sent just before a disconnect no longer runs when the next phone connects (#1).
- During an obstacle block, HORN and FORWARD now silence the warning buzz. FORWARD stays blocked until the robot moves away (#1).

## [1.0.0] - 2026-03-19

The firmware as I submitted it with the bachelor thesis (June 2025), published on GitHub.

### Added

- MicroPython firmware for the Raspberry Pi Pico W: BLE remote control, ultrasonic obstacle avoidance, wheel-encoder odometry, and feed-forward plus PID motion control.
- Thesis PDF, README, a lint workflow, and the MIT license.

[Unreleased]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.3.0...HEAD
[1.3.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.1.1...v1.2.0
[1.1.1]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/andrew-abdelmalak/smart-cleaning-robot/releases/tag/v1.0.0
