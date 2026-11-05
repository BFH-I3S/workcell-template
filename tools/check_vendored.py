#!/usr/bin/env python3
"""
Check that the vendored third-party packages in src/ are still vanilla.

Reads third_party/vendored.yaml and compares every package marked
"vanilla" file by file with the recorded SHA-256 hashes: a changed, missing
or added file is reported and the exit status is 1. Local adaptations belong
in wrappers outside the package (src/cell_description), not in it.

Usage:  python3 tools/check_vendored.py            (from the workspace root)
"""
import hashlib
import pathlib
import sys

import yaml

ROOT = pathlib.Path(__file__).resolve().parent.parent


def main():
    manifest = yaml.safe_load((ROOT / "third_party" / "vendored.yaml").read_text())
    failures = 0
    for name, entry in manifest["packages"].items():
        if entry.get("status") != "vanilla":
            print(f"{name}: {entry.get('status')} - not checked ({entry.get('note', '').strip()[:60]}...)")
            continue
        path = ROOT / entry["path"]
        expected = entry["files"]
        actual = {str(p.relative_to(path)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in path.rglob("*") if p.is_file()}
        changed = sorted(f for f in expected if f in actual and actual[f] != expected[f])
        missing = sorted(set(expected) - set(actual))
        added = sorted(set(actual) - set(expected))
        if changed or missing or added:
            failures += 1
            print(f"{name}: NOT vanilla (upstream {entry['upstream']} @ {entry['commit']})")
            for label, files in (("changed", changed), ("missing", missing), ("added", added)):
                for f in files:
                    print(f"  {label}: {f}")
        else:
            print(f"{name}: vanilla ({len(expected)} files, upstream @ {entry['commit']})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
