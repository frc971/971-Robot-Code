"""Generates index.html with the right checksum for main_bundle_file.js filled in."""

import argparse
import hashlib
import sys
from pathlib import Path

def compute_sha256(filepath):
    return hashlib.sha256(filepath.read_bytes()).hexdigest()

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", type=str)
    parser.add_argument("--bundle", type=str)
    parser.add_argument("--output", type=str)
    args = parser.parse_args(argv[1:])

    template = Path(args.template).read_text()
    bundle_path = Path(args.bundle)
    bundle_sha256 = compute_sha256(bundle_path)

    output = template.format(
        MAIN_BUNDLE_FILE = f"/sha256/{bundle_sha256}/{bundle_path.name}",
    )
    Path(args.output).write_text(output)

if __name__ == "__main__":
    sys.exit(main(sys.argv))
