"""Checks that scripts/vendor.json pins the versions package.json names.

Dependabot bumps package.json; this test then fails until
scripts/fetch_vendor.py --update re-pins the files and their hashes.
"""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_vendored_files_match_package_json() -> None:
    wanted = json.loads((ROOT / "package.json").read_text(encoding="utf-8"))
    manifest = json.loads((ROOT / "scripts/vendor.json").read_text(encoding="utf-8"))
    for entry in manifest["files"]:
        assert re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), entry["path"]
        if entry["package"] in wanted["dependencies"]:
            assert entry["version"] == wanted["dependencies"][entry["package"]], (
                f"{entry['path']} pins {entry['package']}@{entry['version']}; "
                "run: python3 scripts/fetch_vendor.py --update"
            )
