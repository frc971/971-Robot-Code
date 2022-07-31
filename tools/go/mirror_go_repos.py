"""This script mirrors the dependencies from go_deps.bzl as Build-Dependencies.

We use "go mod download" to manually download each Go dependency. We then tar
up all the dependencies and copy them to the Build-Dependencies server for
hosting.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
from typing import List, Dict

# Need a fully qualified import here because @bazel_tools interferes.
import org_frc971.tools.go.mirror_lib

GO_DEPS_WWWW_DIR = "/var/www/html/files/frc971/Build-Dependencies/go_deps"


def compute_sha256(filepath: str) -> str:
    """Computes the SHA256 of a file at the specified location."""
    with open(filepath, "rb") as file:
        contents = file.read()
    return hashlib.sha256(contents).hexdigest()


def get_existing_mirrored_repos(ssh_host: str) -> Dict[str, str]:
    """Gathers information about the libraries that are currently mirrored."""
    run_result = subprocess.run(
        ["ssh", ssh_host, f"bash -c 'sha256sum {GO_DEPS_WWWW_DIR}/*'"],
        check=True,
        stdout=subprocess.PIPE)

    existing_mirrored_repos = {}
    for line in run_result.stdout.decode("utf-8").splitlines():
        sha256, fullpath = line.split()
        existing_mirrored_repos[Path(fullpath).name] = sha256

    return existing_mirrored_repos


def download_repos(repos: Dict[str, str], existing_mirrored_repos: Dict[str,
                                                                        str],
                   tar: tarfile.TarFile) -> Dict[str, str]:
    """Downloads the not-yet-mirrored repos into a tarball."""
    cached_info = {}

    for repo in repos:
        print(f"Downloading file for {repo['name']}")
        importpath = repo["importpath"]
        version = repo["version"]
        module = f"{importpath}@{version}"

        download_result = subprocess.run(
            ["external/go_sdk/bin/go", "mod", "download", "-json", module],
            check=True,
            stdout=subprocess.PIPE)
        if download_result.returncode != 0:
            print("Failed to download file.")
            return 1

        module_info = json.loads(download_result.stdout.decode("utf-8"))

        name = repo["name"]
        zip_path = Path(module_info["Zip"])
        mirrored_name = f"{name}__{zip_path.name}"
        if mirrored_name not in existing_mirrored_repos:
            # We only add the Go library to the tarball if it's not already
            # mirrored. We don't want to overwrite files.
            tar.add(zip_path, arcname=mirrored_name)
            sha256 = compute_sha256(zip_path)
        else:
            # Use the already-computed checksum for consistency.
            sha256 = existing_mirrored_repos[mirrored_name]

        cached_info[name] = {
            "strip_prefix": module,
            "filename": mirrored_name,
            "sha256": sha256,
            "version": version,
            "importpath": importpath,
        }

    return cached_info


def copy_to_host_and_unpack(filename: str, ssh_host: str) -> None:
    subprocess.run(["scp", filename, f"{ssh_host}:"], check=True)

    # Be careful not to use single quotes in these commands to avoid breaking
    # the subprocess.run() invocation below.
    command = " && ".join([
        f"tar -C {GO_DEPS_WWWW_DIR} --no-same-owner -xvaf {filename}",
        # Change the permissions so other users can read them (and checksum
        # them).
        f"find {GO_DEPS_WWWW_DIR}/ -type f -exec chmod 644 {{}} +",
    ])

    print("You might be asked for your sudo password shortly.")
    subprocess.run(
        ["ssh", "-t", ssh_host, f"sudo -u www-data bash -c '{command}'"],
        check=True)


def main(argv):
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--prune",
        action="store_true",
        help=("When set, makes the tool prune go_mirrors_bzl to match the "
              "repositories specified in go_deps_bzl. Incompatible with "
              "--ssh_host."))
    group.add_argument(
        "--ssh_host",
        type=str,
        help=("The SSH host to copy the downloaded Go repositories to. This "
              "should be software.971spartans.net where all the "
              "Build-Dependencies files live. Only specify this if you have "
              "access to the server."))
    parser.add_argument("--go_deps_bzl", type=str, default="go_deps.bzl")
    parser.add_argument("--go_mirrors_bzl",
                        type=str,
                        default="tools/go/go_mirrors.bzl")
    args = parser.parse_args(argv[1:])

    os.chdir(os.environ["BUILD_WORKSPACE_DIRECTORY"])

    repos = org_frc971.tools.go.mirror_lib.parse_go_repositories(
        args.go_deps_bzl)

    if args.ssh_host:
        existing_mirrored_repos = get_existing_mirrored_repos(args.ssh_host)
    else:
        existing_mirrored_repos = {}

    exit_code = 0

    if args.prune:
        # Delete all mirror info that is not needed anymore.
        existing_cache_info = org_frc971.tools.go.mirror_lib.parse_go_mirror_info(
            args.go_mirrors_bzl)
        cached_info = {}
        for repo in repos:
            try:
                cached_info[repo["name"]] = existing_cache_info[repo["name"]]
            except KeyError:
                print(f"{repo['name']} needs to be mirrored still.")
                exit_code = 1
    else:
        # Download all the repositories that need to be mirrored.
        with tarfile.open("go_deps.tar", "w") as tar:
            cached_info = download_repos(repos, existing_mirrored_repos, tar)
            num_not_already_mirrored = len(tar.getnames())

        print(f"Found {num_not_already_mirrored}/{len(cached_info)} libraries "
              "that need to be mirrored.")

        # Only mirror the deps if we've specified an SSH host and we actually have
        # something to mirror.
        if args.ssh_host and num_not_already_mirrored:
            copy_to_host_and_unpack("go_deps.tar", args.ssh_host)
        else:
            print(
                "Skipping mirroring because of lack of --ssh_host or there's "
                "nothing to actually mirror.")

    org_frc971.tools.go.mirror_lib.write_go_mirror_info(
        args.go_mirrors_bzl, cached_info)

    return exit_code


if __name__ == "__main__":
    sys.exit(main(sys.argv))
