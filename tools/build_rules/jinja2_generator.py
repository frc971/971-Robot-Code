#!/usr/bin/python3

import argparse
import json
import sys
from pathlib import Path

import jinja2


def main():
    # Note: this is a pretty transparent interface to jinja2--there's no reason
    # this script couldn't be renamed and then used to generate any config from
    # a template.
    parser = argparse.ArgumentParser(
        description="Generates the raspberry pi configs from a template.")
    parser.add_argument("template",
                        type=Path,
                        help="File to use for template.")
    parser.add_argument(
        "replacements",
        type=json.loads,
        help="Dictionary of parameters to replace in the template.")
    parser.add_argument(
        "--include_dir",
        action="append",
        type=Path,
        default=[],
        help="One or more search directories for {% include %} blocks.",
    )
    parser.add_argument("output", type=Path, help="Output file to create.")
    args = parser.parse_args(sys.argv[1:])

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.include_dir))
    template = env.from_string(args.template.read_text())

    args.output.write_text(template.render(args.replacements))


if __name__ == '__main__':
    main()
