"""This script generates contents of the @pip repository.

This repository is just a way to have simple-to-write targets to specify in
BUILD files.  E.g. you can use @pip//numpy.

The pip package names are normalized:
- Letters are lowercased.
- Periods are replaced by underscores.
- Dashes are replaced by underscores.

We do this normalization because it produces predictable names. pip allows a
wide range of names to refer to the same package. That would be annoying to use
in BUILD files.
"""

import sys
import textwrap
from pathlib import Path


def parse_requirements(requirements_path: Path) -> list[str]:
    """Parses tools/python/requirements.txt.

    We don't want to parse the lock file since we really only want users to
    depend on explicitly requested pip packages. We don't want users to depend
    on transitive dependencies of our requested pip packages.
    """
    result = []
    for line in requirements_path.read_text().splitlines():
        # Ignore line comments.
        if not line or line.startswith("#"):
            continue

        # Remove any inline comments that may or may not exist.
        # E.g:
        # numpy==1.2.3  # needed because we like it.
        result.append(line.split()[0])

    return result


def generate_build_files(requirements: list[str]) -> None:
    """Generate all the BUILD files in the "pip" external repository.

    We create files like this:

        external/pip/numpy/BUILD

    and in that BUILD file we create a "numpy" target. That lets users depend
    on "@pip//numpy".
    """
    for requirement in requirements:
        requirement = requirement.lower().replace("-", "_").replace(".", "_")
        requirement_dir = Path(requirement)
        requirement_dir.mkdir()
        # We could use an alias() here, but that behaves strangely with
        # target_compatible_with pre-6.0.
        (requirement_dir / "BUILD").write_text(
            textwrap.dedent(f"""\
            load("@pip_deps//:requirements.bzl", "requirement")
            py_library(
                name = "{requirement}",
                deps = [requirement("{requirement}")],
                visibility = ["//visibility:public"],
                target_compatible_with = [
                    "@//tools/platforms/python:upstream_bundled_python",
                ],
            )
            """))


def main(argv):
    requirements_path = Path(argv[1])
    requirements = parse_requirements(requirements_path)

    generate_build_files(requirements)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
