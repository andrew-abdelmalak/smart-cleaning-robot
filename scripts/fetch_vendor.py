"""Fetch the site's third-party browser files from npm and check their hashes.

The site serves MicroPython WebAssembly, transformers.js, and the ONNX
runtime itself, so a changed file on a CDN cannot change the page. Each file
comes from its npm package tarball, which must match the registry's
integrity hash, and must then match the SHA-256 pinned in vendor.json.

    python3 scripts/fetch_vendor.py site/vendor    # what the Pages build runs
    python3 scripts/fetch_vendor.py --update       # after a version bump

package.json holds the versions so Dependabot can propose updates. The ONNX
runtime is not listed there: its version is the one transformers.js depends
on. --update reads the versions, downloads the tarballs, and rewrites the
versions and hashes in vendor.json.
"""

import argparse
import base64
import hashlib
import io
import json
import tarfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = Path(__file__).with_name("vendor.json")
PACKAGE_JSON = ROOT / "package.json"
REGISTRY = "https://registry.npmjs.org"
ORT = "onnxruntime-web"
TRANSFORMERS = "@huggingface/transformers"


def _get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=300) as response:
        return response.read()


def package(name: str, version: str) -> tuple[tarfile.TarFile, dict]:
    """Download a package tarball, verified against the registry's integrity."""
    meta = json.loads(_get(f"{REGISTRY}/{name}/{version}"))
    data = _get(meta["dist"]["tarball"])
    algorithm, _, expected = meta["dist"]["integrity"].partition("-")
    actual = base64.b64encode(hashlib.new(algorithm, data).digest()).decode()
    if actual != expected:
        raise SystemExit(f"{name}@{version}: tarball does not match the registry")
    return tarfile.open(fileobj=io.BytesIO(data)), meta


def _read(tar: tarfile.TarFile, file: str) -> bytes:
    member = tar.extractfile(f"package/{file}")
    if member is None:
        raise SystemExit(f"package/{file} is not a file")
    return member.read()


def fetch(out_dir: Path) -> None:
    """Write every manifest file under out_dir, or fail on any hash mismatch."""
    entries = json.loads(MANIFEST.read_text(encoding="utf-8"))["files"]
    tars: dict[tuple[str, str], tarfile.TarFile] = {}
    for entry in entries:
        target = out_dir / entry["path"]
        if target.exists() and _sha256(target.read_bytes()) == entry["sha256"]:
            continue
        key = (entry["package"], entry["version"])
        if key not in tars:
            tars[key] = package(*key)[0]
        data = _read(tars[key], entry["file"])
        if _sha256(data) != entry["sha256"]:
            raise SystemExit(f"{entry['path']}: sha256 does not match vendor.json")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        print(f"{entry['path']}: {len(data):,} bytes, hash ok")


def update() -> None:
    """Pin the versions in package.json, and their files' hashes, in vendor.json."""
    versions = json.loads(PACKAGE_JSON.read_text(encoding="utf-8"))["dependencies"]
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    tars = {}
    for name, version in list(versions.items()):
        tars[name], meta = package(name, version)
        if name == TRANSFORMERS:
            versions[ORT] = meta["dependencies"][ORT]
    tars[ORT] = package(ORT, versions[ORT])[0]
    for entry in manifest["files"]:
        entry["version"] = versions[entry["package"]]
        entry["sha256"] = _sha256(_read(tars[entry["package"]], entry["file"]))
        print(f"{entry['path']}: {entry['package']}@{entry['version']}")
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out_dir", nargs="?", type=Path, help="where to write")
    parser.add_argument("--update", action="store_true", help="re-pin vendor.json")
    args = parser.parse_args()
    if args.update:
        update()
    elif args.out_dir:
        fetch(args.out_dir)
    else:
        parser.error("give OUT_DIR, or --update")
