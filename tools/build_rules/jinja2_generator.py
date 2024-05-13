#!/usr/bin/python3

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import jinja2


def load_filter_file(filename: Path, env: jinja2.Environment):
    """Adds filters specified in the .py file.

    The .py file has to define a `register_filters` function that will be
    invoked. The function will be passed the jinja2 environment.

        def register_filters(env: jinja2.Environment):
            env.filters["custom_filter"] = ...

    Then you can use it in the template.

        Hello {{ "world" | custom_filter }}!

    Based on https://stackoverflow.com/a/51575312.

    Args:
        filename: The .py file to load.
        env: The environment to pass to the `register_filters` function.
    """
    spec = importlib.util.spec_from_file_location("filter_module", filename)
    filter_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(filter_module)
    filter_module.register_filters(env)


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
        "--replacements_file",
        type=Path,
        help=("File containing a dictionary of parameters to replace "
              "in the template. The behaviour is undefined if keys are "
              "duplicated between this file and the `replacements` argument."),
    )
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
    parser.add_argument(
        "--filter_file",
        action="append",
        type=str,
        default=[],
        help=("A .py file with a register_filters() function for custom "
              "jinja2 filters."),
    )
    args = parser.parse_args(sys.argv[1:])

    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.include_dir))
    for filename in args.filter_file:
        load_filter_file(filename, env)

    template = env.from_string(args.template.read_text())

    replacements = args.replacements.copy()
    if args.replacements_file:
        with args.replacements_file.open() as file:
            replacements.update(json.load(file))

    args.output.write_text(template.render(replacements))


if __name__ == '__main__':
    main()
