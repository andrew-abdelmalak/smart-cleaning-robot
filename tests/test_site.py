"""Checks that the project site's references to the repository stay true.

The site ships files the Pages workflow copies in (the firmware, the thesis
PDF, Mozilla's PDF viewer) and copies a few numbers from the firmware. These
tests fail when either side changes without the other.
"""

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
