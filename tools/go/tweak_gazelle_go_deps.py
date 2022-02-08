"""Tweaks the gazelle-generated go_deps.bzl to work with our mirrors.

This script changes all invocations of go_repository() in go_deps.bzl to use
maybe_override_go_dep(). That lets us more easily switch between upstream
sources and our mirrored versions of the code.

The motivation is to let folks use upstream sources during local development.
For CI runs, however, we have to restrict ourselves to mirrored dependencies.
"""

import argparse
import sys
import textwrap

import org_frc971.tools.go.mirror_lib

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("go_deps_bzl", type=str)
    args = parser.parse_args(argv[1:])

    repos = org_frc971.tools.go.mirror_lib.parse_go_repositories(args.go_deps_bzl)

    with open(args.go_deps_bzl, "w") as file:
        file.write(textwrap.dedent("""\
            # This file is auto-generated. Do not edit.
            load("//tools/go:mirrored_go_deps.bzl", "maybe_override_go_dep")

            def go_dependencies():
            """))
        for repo in repos:
            file.write(textwrap.indent(textwrap.dedent(f"""\
                maybe_override_go_dep(
                    name = "{repo['name']}",
                    importpath = "{repo['importpath']}",
                    sum = "{repo['sum']}",
                    version = "{repo['version']}",
                )
                """), " " * 4))

if __name__ == "__main__":
    sys.exit(main(sys.argv))
