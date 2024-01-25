import argparse
import shutil
import sys
from pathlib import Path


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, action="append", default=[])
    parser.add_argument("--relative_output",
                        type=Path,
                        action="append",
                        default=[])
    args = parser.parse_args(argv[1:])

    for relative_output, output in zip(args.relative_output, args.output):
        for input_dir in args.input_dir:
            input_file = input_dir / relative_output
            if input_file.exists():
                print(f"Copying {input_file} to {output}")
                shutil.copy(input_file, output)
                break


if __name__ == "__main__":
    sys.exit(main(sys.argv))
