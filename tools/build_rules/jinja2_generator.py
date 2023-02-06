#!/usr/bin/python3

import argparse
import json
import sys

import jinja2


def main():
    # Note: this is a pretty transparent interface to jinja2--there's no reason
    # this script couldn't be renamed and then used to generate any config from
    # a template.
    parser = argparse.ArgumentParser(
        description="Generates the raspberry pi configs from a template.")
    parser.add_argument("template", type=str, help="File to use for template.")
    parser.add_argument(
        "replacements",
        type=json.loads,
        help="Dictionary of parameters to replace in the template.")
    parser.add_argument("output", type=str, help="Output file to create.")
    args = parser.parse_args(sys.argv[1:])

    with open(args.template, 'r') as input_file:
        template = jinja2.Environment(
            loader=jinja2.FileSystemLoader(".")).from_string(input_file.read())

    output = template.render(args.replacements)
    with open(args.output, 'w') as config_file:
        config_file.write(output)


if __name__ == '__main__':
    main()
