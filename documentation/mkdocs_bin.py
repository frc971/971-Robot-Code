#!/usr/bin/env python3
"""This is a small wrapper to allow bazel to call mkdocs as a script.

Because the mkdocs library handles consuming command-line arguments directly,
we (rather than attempting to mess with arguments ourselves), use environment
variables to communicate certain flags. Namely:
If the OUTPUT environment variable is set, then we will tar up the output of
the SITE_DIR directory into the OUTPUT file. This is used to make the mkdocs
output bazel-friendly by only outputting a single file.
"""
from mkdocs.__main__ import cli
import os
import sys
import tarfile
try:
    cli()
except SystemExit as err:
    if err.code != 0:
        raise err
if "OUTPUT" in os.environ:
    with tarfile.open(os.environ["OUTPUT"], "w") as tarball:
        tarball.add(os.environ["SITE_DIR"], arcname="")
