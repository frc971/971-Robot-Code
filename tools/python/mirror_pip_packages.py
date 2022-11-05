"""This script mirrors our pip package dependencies.

This script looks at the requirements.lock.txt file and generate a wheel for
each entry. Those wheels are then mirrored.

See tools/python/README.md for some more information.
"""

import argparse
import hashlib
import json
import os
import pwd
import subprocess
import sys
import tarfile
from pathlib import Path
from typing import List, Optional, Tuple

import requests
from pkginfo import Wheel

PLAT = "manylinux_2_31"
ARCH = "x86_64"
WHEELHOUSE_MIRROR_URL = "https://software.frc971.org/Build-Dependencies/wheelhouse"
PY_DEPS_WWWW_DIR = "/var/www/html/files/frc971/Build-Dependencies/wheelhouse"


def compute_sha256(data: bytes) -> str:
    """Computes the sha256 checksum of a bytes sequence.

    Args:
        data: The bytes to checksum.

    Returns:
        The hex representation of the checksum.
    """
    hasher = hashlib.sha256()
    hasher.update(data)
    return hasher.hexdigest()


def compute_file_sha256(filename: Path) -> str:
    """Computes the sha256 checksum of the content of a file.

    Args:
        filename: The file to checksum.

    Returns:
        The hex representation of the checksum.
    """
    return compute_sha256(filename.read_bytes())


def search_for_uploaded_wheel(wheel: Path, wheel_url: str) -> Tuple[bool, str]:
    """Searches for this wheel on our internal mirror.

    Since we can't build wheels reproducibly, our best option is to check
    whether this wheel already exists on the mirror. If it does, we can skip
    uploading it.

    Args:
        wheel: The wheel to search for on the mirror.
        wheel_url: The URL where the wheel is expected if it exists on the mirror.

    Returns:
        A two-tuple. The first value is a boolean that signifies whether the
        wheel was found on the mirror. The second value is a string. If the
        wheel was not found on the mirror, this is an empty string. Otherwise,
        this string contains the sha256 checksum of the wheel found on the
        mirror.
    """
    # TODO(phil): A better way to do this would be to SSH into the host and
    # look for files on the filesystem.
    request = requests.get(wheel_url)

    if request.status_code == 200:
        return True, compute_sha256(request.content)
    if request.status_code == 404:
        return False, ""

    raise RuntimeError(
        f"Don't know what to do with status code {request.status_cdoe} when trying to get {wheel_url}"
    )


def copy_to_host_and_unpack(filename: str, ssh_host: str) -> None:
    """Copies the tarball of wheels to the server and unpacks the tarball.

    Args:
        filename: The path to the tarball to be uploaded.
        ssh_host: The server that will be passed to ssh(1) for uploading and
            unpacking the tarball.
    """
    # TODO(phil): De-duplicate with tools/go/mirror_go_repos.py

    subprocess.run(["scp", filename, f"{ssh_host}:"], check=True)

    # Be careful not to use single quotes in these commands to avoid breaking
    # the subprocess.run() invocation below.
    command = " && ".join([
        f"mkdir -p {PY_DEPS_WWWW_DIR}",
        f"tar -C {PY_DEPS_WWWW_DIR} --no-same-owner -xvaf {filename.name}",
        # Change the permissions so other users can read them (and checksum
        # them).
        f"find {PY_DEPS_WWWW_DIR}/ -type f -exec chmod 644 {{}} +",
    ])

    print("You might be asked for your sudo password shortly.")
    subprocess.run(
        ["ssh", "-t", ssh_host, f"sudo -u www-data bash -c '{command}'"],
        check=True)


def main(argv: List[str]) -> Optional[int]:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help=("If set, ignores packages we have already uploaded and "
              "possibly overwrite them with the just-built ones. Use with "
              "extreme caution! This may easily cause issues with building "
              "older commits. Use this only if you know what you're doing."))
    parser.add_argument(
        "-l",
        "--local_test",
        action="store_true",
        help=("If set, generate the URL overrides pointing at the generated "
              "local files. Incompatible with --ssh_host. This is useful for "
              "iterating on generated wheel files."))
    parser.add_argument(
        "--ssh_host",
        type=str,
        help=("The SSH host to copy the downloaded Go repositories to. This "
              "should be software.971spartans.net where all the "
              "Build-Dependencies files live. Only specify this if you have "
              "access to the server."))
    args = parser.parse_args(argv[1:])

    root_dir = Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
    caller = os.getenv("SUDO_USER") or os.environ["USER"]
    caller_id = pwd.getpwnam(caller).pw_uid

    python_dir = root_dir / "tools" / "python"

    container_tag = f"pip-compile:{caller}"

    subprocess.run([
        "docker",
        "build",
        "--file=generate_pip_packages.Dockerfile",
        f"--tag={container_tag}",
        ".",
    ],
                   cwd=python_dir,
                   check=True)

    # Run the wheel generation script inside the docker container provided by
    # the pypa/manylinux project.
    # https://github.com/pypa/manylinux/
    subprocess.run([
        "docker",
        "run",
        "-it",
        "-v",
        f"{python_dir}:/opt/971_build/",
        container_tag,
        "/opt/971_build/generate_pip_packages_in_docker.sh",
        PLAT,
        ARCH,
        str(caller_id),
    ],
                   check=True)

    # Get the list of wheels we downloaded form pypi.org or built ourselves.
    wheelhouse = python_dir / "wheelhouse"
    wheels = wheelhouse.glob("*.whl")

    # Assemble the override list. This list will tell rules_python to download
    # from our mirror instead of pypi.org.
    wheels_to_be_uploaded = []
    override_information = {}
    for wheel in sorted(wheels):
        wheel_url = f"{WHEELHOUSE_MIRROR_URL}/{wheel.name}"
        if args.local_test:
            override_url = f"file://{wheel.resolve()}"
        else:
            override_url = wheel_url
        sha256 = compute_file_sha256(wheel)

        # Check if we already have the wheel uploaded. If so, download that one
        # into the wheelhouse. This lets us avoid non-reproducibility with pip
        # and native extensions.
        # https://github.com/pypa/pip/issues/9604
        wheel_found, sha256_on_mirror = search_for_uploaded_wheel(
            wheel, wheel_url)

        if args.local_test:
            wheel_found = False

        if args.force:
            if wheel_found and sha256 != sha256_on_mirror:
                print(
                    f"WARNING: The next upload will change sha256 for {wheel}!"
                )
            wheels_to_be_uploaded.append(wheel)
        else:
            if wheel_found:
                sha256 = sha256_on_mirror
            else:
                wheels_to_be_uploaded.append(wheel)

        # Update the override information for this wheel.
        # We use lower-case for the package names here because that's what the
        # requirements.lock.txt file uses.
        info = Wheel(wheel)
        override_information[f"{info.name.lower()}=={info.version}"] = {
            "url": override_url,
            "sha256": sha256,
        }

    print(f"We need to upload {len(wheels_to_be_uploaded)} wheels:")
    for wheel in wheels_to_be_uploaded:
        print(wheel)

    # Create a tarball of all the wheels that need to be mirrored.
    py_deps_tar = root_dir / "py_deps.tar"
    with tarfile.open(py_deps_tar, "w") as tar:
        for wheel in wheels_to_be_uploaded:
            tar.add(wheel, arcname=wheel.name)

    # Upload the wheels if requested.
    if wheels_to_be_uploaded and args.ssh_host:
        copy_to_host_and_unpack(py_deps_tar, args.ssh_host)
    else:
        print("Skipping mirroring because of lack of --ssh_host or there's "
              "nothing to actually mirror.")

    # Write out the overrides file.
    override_file = python_dir / "whl_overrides.json"
    override_file.write_text(
        json.dumps(override_information, indent=4, sort_keys=True) + "\n")


if __name__ == "__main__":
    sys.exit(main(sys.argv))
