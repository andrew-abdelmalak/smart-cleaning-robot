# Firmware history

Three earlier versions of the robot's MicroPython firmware, kept as I wrote them during the thesis, so the path to [`firmware/main.py`](../../firmware/main.py) is visible. Thesis section 4.5.2 describes the same steps.

These files are not maintained, tested, or linted. Only their line endings changed (Windows to Unix). The dates come from the original files.

| Step | File | Date | How it drives |
|---|---|---|---|
| 1 | [`1-time-based.py`](1-time-based.py) | May 2025 | Open loop, on timers. Fixed motor duties (left 65%, right 67.1%) keep it roughly straight. At an obstacle under 16 cm it pauses 1.5 s, turns for a fixed 1.25 s, and shifts forward for 0.4 s. It has no wheel encoders. |
| 2 | [`2-velocity-pid.py`](2-velocity-pid.py) | 12 June 2025 | Closed loop. The wheel encoders arrive: each wheel has its own velocity PID toward 100 pulses per second, with the power change capped at 3.5 per sample. Turns and reversing count pulses (50 for a turn). A stall is a wheel at over 95% power below 10 pulses per second for 1 s. There is no feed-forward yet. |
| 3 | [`3-straight-line-tuner.py`](3-straight-line-tuner.py) | 12 June 2025 | A test program, not a full firmware. It drives straight for 59 pulses (about 60 cm) with a positional PID on the pulse difference and a right-wheel trim lowered from 0.8 to 0.75. Those gains (P 0.8, I 0.15, D 0.25), the 0.75 trim, and the 50 ms brake are the ones the final firmware uses. |
| 4 | [`firmware/main.py`](../../firmware/main.py) | June 2025 | The submitted firmware: feed-forward from a per-wheel motor model supplies most of the drive, and PID adds small corrections. See the [README](../../README.md#motion-control). |

A copy of step 1 with its docstrings and debug prints removed also existed; it runs the same logic, so it is not included.
