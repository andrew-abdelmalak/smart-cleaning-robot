# Contributing to Smart Cleaning Robot

This is a finished bachelor thesis project. Bug reports, bug fixes, and documentation corrections are welcome. I do not plan new features, so please open an issue before you start on anything larger than a small fix.

## Before you start

- Search [existing issues](https://github.com/andrew-abdelmalak/smart-cleaning-robot/issues) to avoid duplicates.
- By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Set up your environment

You need Python 3.12 or later and [uv](https://docs.astral.sh/uv/). The firmware itself runs on MicroPython, but the tests run on your computer with fake hardware modules, so you do not need a robot.

```bash
git clone https://github.com/andrew-abdelmalak/smart-cleaning-robot.git
cd smart-cleaning-robot
uv sync
```

## Run the checks

```bash
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv run mpy-cross -o main.mpy firmware/main.py
```

CI runs the same checks on every pull request, and a pull request with failing checks is not merged.

## Build the thesis

The LaTeX source in `docs/thesis/` mirrors the [Overleaf project](https://www.overleaf.com/read/hzhdgbtyqgmt#95d41f), which builds with pdfLaTeX on TeX Live 2025. To build it locally with the same engine, run this from the repository root:

```bash
docker run --rm -v "$PWD/docs/thesis":/work -w /work texlive/texlive:TL2025-historic latexmk -pdf main.tex
```

The Thesis workflow runs the same build on every change to `docs/thesis/`.

## Make your change

1. Create a branch from `main`, for example `git switch -c fix-horn-toggle`.
2. Keep the change focused on one problem.
3. Add a test that fails without your change. The fakes in `tests/conftest.py` cover the motors, encoders, ultrasonic sensor, button, LED, and BLE.
4. Do not retune the PID gains, the feed-forward constants, or the thresholds without testing on hardware, and say in the pull request how you tested.
5. Update the README when behavior or commands change.

## Commit messages

Write a short summary in the imperative mood, under 72 characters, for example `Clear the obstacle block in emergency_stop`. Explain the reason in the body when it is not obvious.

## Open a pull request

1. Push your branch and open a pull request against `main`.
2. Fill in the pull request template, and link the issue it resolves (`Fixes #123`).

## Report a bug

Use the [issue forms](https://github.com/andrew-abdelmalak/smart-cleaning-robot/issues/new/choose). For security problems, follow [SECURITY.md](SECURITY.md) instead, and never report them in a public issue.

## License

By contributing, you agree that your contributions are licensed under the project's [MIT License](../LICENSE).
