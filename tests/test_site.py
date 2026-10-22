"""Checks that the project site's references to the repository stay true.

The site ships files the Pages workflow copies in (the firmware, the thesis
PDF, Mozilla's PDF viewer) and copies a few numbers from the firmware. These
tests fail when either side changes without the other.
"""

import ast
import contextlib
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SITE = ROOT / "site"
PAGES = (ROOT / ".github/workflows/pages.yml").read_text(encoding="utf-8")
VIEWER = "pdfjs/web/viewer.html?file=../../thesis.pdf"


def test_local_assets_exist() -> None:
    html = (SITE / "index.html").read_text(encoding="utf-8")
    copied = {  # what the Pages workflow adds next to site/
        "media/": ROOT / "docs/media",
        "diagrams/": ROOT / "docs/diagrams",
        "vendor/": None,  # fetched by scripts/fetch_vendor.py
    }
    for ref in re.findall(r'(?:src|href)="([^"#:]+)"', html):
        path = ref.partition("?")[0]
        if path == "pdfjs/web/viewer.html":
            assert ref == VIEWER, ref
            continue
        prefix = next((p for p in copied if path.startswith(p)), None)
        if prefix is None:
            assert (SITE / path).exists(), ref
        elif copied[prefix] is not None:
            assert (copied[prefix] / path.removeprefix(prefix)).exists(), ref


def test_thesis_opens_in_the_pinned_viewer() -> None:
    ask = (SITE / "ask.js").read_text(encoding="utf-8")
    assert f"`{VIEWER}#page=${{p.pdfPage}}&search=${{search}}&phrase=true`" in ask
    assert "cp docs/thesis/thesis.pdf _site/" in PAGES
    assert "PDFJS_VERSION: 6.4.299" in PAGES
    assert re.search(r"PDFJS_SHA256: [0-9a-f]{64}\n", PAGES)
    assert "sha256sum -c" in PAGES
    assert "unzip -q pdfjs.zip -d _site/pdfjs" in PAGES


def _firmware_constants() -> dict:
    """Module-level constants of firmware/main.py, read without importing it."""
    tree = ast.parse((ROOT / "firmware/main.py").read_text(encoding="utf-8"))
    values = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            with contextlib.suppress(ValueError):  # hardware objects and calls
                values[node.targets[0].id] = ast.literal_eval(node.value)
    return values


def test_page_constants_match_the_firmware() -> None:
    """Every number the page copies from the firmware still matches it."""
    fw = _firmware_constants()
    firmware = (ROOT / "firmware/main.py").read_text(encoding="utf-8")
    bench = (SITE / "bench.js").read_text(encoding="utf-8")
    html = (SITE / "index.html").read_text(encoding="utf-8")

    copied = re.search(r"const FW = \{([^}]+)\}", bench).group(1)
    pairs = dict(re.findall(r"(\w+): (\d+)", copied))
    assert pairs, "bench.js has no FW constants"
    for name, value in pairs.items():
        assert int(value) == fw[name], name

    # The text beside the state machine and the trace.
    note = re.search(r'<p class="fsm-note">(.+?)</p>', html, re.DOTALL).group(1)
    assert f"under {fw['OBSTACLE_THRESHOLD_CM']} cm" in note
    assert f"for {fw['STALL_TIME_MS']} ms" in note
    assert f"reverse for {fw['STALL_REVERSE_MS']} ms" in note
    assert f"pivot left {fw['TURN_PULSES']} pulses" in note
    assert f"the {fw['OBSTACLE_THRESHOLD_CM']} cm stop line" in html
    assert "ena.freq(1000)" in firmware and "the pins switch at 1 kHz" in html

    # The state chips name the firmware's own states and commands.
    chips = set(re.findall(r'data-state="(\w+)"', html))
    states = set(re.findall(r'autonomous_state = "(\w+)"', firmware))
    commands = {"FORWARD", "BACKWARD", "LEFT", "RIGHT"}
    assert chips == states | commands | {"HORN", "BLOCKED"}, chips

    # The For engineers section: loop period, diagram, and gains.
    assert firmware.count("time.sleep_ms(20)") == 2 and "a 20 ms control loop" in html
    assert (
        f"obstacle closer than {fw['OBSTACLE_THRESHOLD_CM']} cm triggers "
        f"a {fw['TURN_PULSES']}-pulse left pivot" in html
    )
    assert f"wheel stall longer than {fw['STALL_TIME_MS']} ms" in html
    assert f"Reach the {fw['TURN_PULSES']}-pulse pivot" in html
    for loop, gains in (
        ("Straight", fw["GAINS_STRAIGHT"]),
        ("Turn progress", fw["GAINS_TURN_PROGRESS"]),
        ("Turn balance", fw["GAINS_TURN_BALANCE"]),
    ):
        row = re.search(rf'<th scope="row">{loop}</th>.*?</tr>', html).group(0)
        assert re.findall(r"<td>([\d.]+)</td>", row) == [
            str(gains[k]) for k in ("P", "I", "D")
        ], loop
