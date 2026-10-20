"""Download the site's third-party browser files and check their SHA-256 hashes.

The site serves MicroPython WebAssembly, transformers.js, and the ONNX
runtime itself instead of loading them from a CDN, so a changed file on the
CDN cannot change the page. The Pages build runs this; for a local preview:

    python3 scripts/fetch_vendor.py site/vendor
"""

import hashlib
import json
import sys
import urllib.request
from pathlib import Path

MANIFEST = Path(__file__).with_name("vendor.json")


def fetch(out_dir: Path) -> None:
    """Write every file in the manifest under out_dir, or fail on a hash mismatch."""
    for entry in json.loads(MANIFEST.read_text(encoding="utf-8"))["files"]:
        target = out_dir / entry["path"]
        if target.exists() and _sha256(target.read_bytes()) == entry["sha256"]:
            continue
        with urllib.request.urlopen(entry["url"], timeout=120) as response:
            data = response.read()
        digest = _sha256(data)
        if digest != entry["sha256"]:
            raise SystemExit(
                f"{entry['url']}: sha256 {digest}, expected {entry['sha256']}"
            )
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        print(f"{entry['path']}: {len(data):,} bytes, hash ok")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: fetch_vendor.py OUT_DIR")
    fetch(Path(sys.argv[1]))
