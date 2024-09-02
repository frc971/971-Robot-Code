"""Acts as a dummy `npm` binary for the `create-foxglove-extension` binary.

The `create-foxglove-extension` binary uses `npm` to manipulate the
`package.json` file instead of doing so directly. Since we don't have access to
the real `npm` binary here we just emulate the limited functionality we need.
"""

import argparse
import json
import sys
from pathlib import Path


def main(argv: list[str]):
    """Runs the main logic."""
    parser = argparse.ArgumentParser()
    parser.add_argument("command")
    parser.add_argument("--save-exact", action="store_true")
    parser.add_argument("--save-dev", action="store_true")
    args, packages = parser.parse_known_args(argv[1:])

    # Validate the input arguments.
    if args.command != "install":
        raise ValueError("Don't know how to simulate anything other "
                         f"than 'install'. Got '{args.command}'.")

    for package in packages:
        if "@^" not in package:
            raise ValueError(f"Got unexpected package: {package}")

    # Append the specified packages to the dependencies list.
    package_version_pairs = list(
        package.rsplit("@", maxsplit=1) for package in packages)
    package_json_file = Path.cwd() / "package.json"
    package_json = json.loads(package_json_file.read_text())
    package_json.setdefault("dependencies", {}).update({
        package: version
        for package, version in package_version_pairs
    })

    package_json_file.write_text(
        json.dumps(package_json, sort_keys=True, indent=4))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
