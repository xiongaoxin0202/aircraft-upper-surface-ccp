#!/usr/bin/env python3
"""Restore large files that are attached to the GitHub release.

The Git tree intentionally excludes a few large binaries.  They are attached to
the ``r1`` release of this repository and can be restored to their canonical
paths with:

    python tools/fetch_release_assets.py

The script verifies both the size and the SHA-256 digest before replacing a
target file.  Use ``--force`` to re-download files that are already present.
"""

import argparse
import hashlib
import json
import os
import sys
import tempfile
import time
import urllib.parse
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(ROOT, "release_assets.json")
USER_AGENT = "aircraft-upper-surface-ccp/1.0 (+https://github.com/xiongaoxin0202/aircraft-upper-surface-ccp)"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url, target, expected_size):
    os.makedirs(os.path.dirname(target), exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".download_", dir=os.path.dirname(target))
    os.close(fd)
    try:
        request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(request, timeout=600) as response, open(tmp, "wb") as out:
            total = int(response.headers.get("Content-Length") or expected_size or 0)
            done = 0
            last_pct = -1
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                out.write(chunk)
                done += len(chunk)
                if total:
                    pct = int(done * 100 / total)
                    if pct >= last_pct + 10:
                        print(f"    {pct:3d}%  {done/1e6:6.2f} / {total/1e6:6.2f} MB", flush=True)
                        last_pct = pct
        os.replace(tmp, target)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="re-download even if the file is valid")
    args = parser.parse_args()

    with open(MANIFEST, encoding="utf-8") as f:
        spec = json.load(f)

    repo = spec["repository"]
    tag = spec["tag"]
    failures = 0
    for asset in spec["assets"]:
        rel = asset["path"]
        target = os.path.join(ROOT, rel)
        name = asset["name"]
        if os.path.exists(target) and not args.force:
            if os.path.getsize(target) == asset["size"] and sha256(target) == asset["sha256"]:
                print(f"[ok]   {rel} (already present)", flush=True)
                continue
        url = (
            f"https://github.com/{repo}/releases/download/{urllib.parse.quote(tag)}/"
            f"{urllib.parse.quote(name)}"
        )
        print(f"[get]  {rel}  <-  {url}", flush=True)
        last_error = None
        for attempt in range(1, 4):
            try:
                download(url, target, asset["size"])
                last_error = None
                break
            except Exception as exc:
                last_error = exc
                print(f"       attempt {attempt}/3 failed: {exc}", flush=True)
                if attempt < 3:
                    time.sleep(5)
        if last_error is not None:
            print(f"[fail] {rel}: {last_error}", flush=True)
            failures += 1
            continue
        size = os.path.getsize(target)
        digest = sha256(target)
        if size != asset["size"] or digest != asset["sha256"]:
            print(
                f"[fail] {rel}: verification failed "
                f"(size {size} vs {asset['size']}, sha256 {digest} vs {asset['sha256']})",
                flush=True,
            )
            failures += 1
        else:
            print(f"[ok]   {rel} ({size/1e6:.2f} MB, SHA-256 verified)", flush=True)

    if failures:
        print(f"{failures} asset(s) could not be restored.", file=sys.stderr)
        return 1
    print("All large files restored and verified.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
