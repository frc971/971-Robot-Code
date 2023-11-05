#!/usr/bin/python3

import contextlib
import datetime
import pathlib
import subprocess
import shlex
import os
import sys

REQUIRED_DEPS = ["debootstrap"]

ROOTFS_FOLDER = "/tmp/rootfs"


@contextlib.contextmanager
def scoped_bind_mount(partition):
    """Bind mounts a folder from the host into the rootfs."""
    result = subprocess.run(
        ["sudo", "mount", "--bind", partition, f"{ROOTFS_FOLDER}/{partition}"],
        check=True)

    try:
        yield partition
    finally:
        subprocess.run(["sudo", "umount", f"{ROOTFS_FOLDER}/{partition}"],
                       check=True)


def check_required_deps(deps):
    """Checks if the provided list of dependencies is installed."""
    missing_deps = []
    for dep in deps:
        result = subprocess.run(["dpkg-query", "-W", "-f='${Status}'", dep],
                                check=True,
                                stdout=subprocess.PIPE)

        if "install ok installed" not in result.stdout.decode('utf-8'):
            missing_deps.append(dep)

    if len(missing_deps) > 0:
        print("Missing dependencies, please install:")
        print("sudo apt-get install", " ".join(missing_deps))
        return True

    return False


def target_unescaped(cmd):
    """Runs a command as root with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=0:0", f"{ROOTFS_FOLDER}", "/bin/bash",
        "-c", cmd
    ],
                   check=True)


def target(cmd):
    """Runs a command as root with escaping."""
    target_unescaped(shlex.join([shlex.quote(c) for c in cmd]))


def copyfile(owner, permissions, file):
    """Copies a file from contents/{file} with the provided owner and permissions."""
    print("copyfile", owner, permissions, file)
    subprocess.run(
        ["sudo", "cp", f"contents/{file}", f"{ROOTFS_FOLDER}/{file}"],
        check=True)
    subprocess.run(["sudo", "chmod", permissions, f"{ROOTFS_FOLDER}/{file}"],
                   check=True)
    target(["chown", owner, f"/{file}"])


def target_symlink(owner, permissions, link_target, linkname):
    full_linkname = f"{ROOTFS_FOLDER}/{linkname}"
    print(link_target)
    print(full_linkname)
    if not os.path.exists(full_linkname):
        target(["ln", "-s", link_target, linkname])

    assert (pathlib.Path(full_linkname).is_symlink())

    target(["chown", owner, linkname])
    target(["chmod", permissions, linkname])


def target_mkdir(owner_group, permissions, folder):
    """Creates a directory recursively with the provided permissions and ownership."""
    print("target_mkdir", owner_group, permissions, folder)
    owner, group = owner_group.split('.')
    target(
        ["install", "-d", "-m", permissions, "-o", owner, "-g", group, folder])


def main():
    if check_required_deps(REQUIRED_DEPS):
        return 1

    new_image = not os.path.exists(ROOTFS_FOLDER)
    if new_image:
        os.mkdir(ROOTFS_FOLDER)

    if new_image:
        subprocess.run([
            "sudo", "debootstrap", "--no-check-gpg", "bookworm", ROOTFS_FOLDER,
            "http://deb.debian.org/debian/"
        ],
                       check=True)

    if not os.path.exists(
            f"{ROOTFS_FOLDER}/etc/apt/sources.list.d/bullseye-backports.list"):
        copyfile("root.root", "644",
                 "etc/apt/sources.list.d/bullseye-backports.list")
        target(["apt-get", "update"])

    with scoped_bind_mount("/dev") as _:
        with scoped_bind_mount("/proc") as _:
            target([
                "apt-get",
                "-y",
                "install",
                "libopencv-calib3d406",
                "libopencv-contrib406",
                "libopencv-core406",
                "libopencv-features2d406",
                "libopencv-flann406",
                "libopencv-highgui406",
                "libopencv-imgcodecs406",
                "libopencv-imgproc406",
                "libopencv-ml406",
                "libopencv-objdetect406",
                "libopencv-photo406",
                "libopencv-shape406",
                "libopencv-stitching406",
                "libopencv-superres406",
                "libopencv-video406",
                "libopencv-videoio406",
                "libopencv-videostab406",
                "libopencv-viz406",
                "libv4l-dev",
                "libc6-dev",
                "libstdc++-12-dev",
                "nvidia-cuda-dev",
                "nvidia-cuda-toolkit",
            ])

    target_mkdir("root.root", "755", "usr/lib/cuda/bin")
    target_symlink("root.root", "555", "../../../bin/fatbinary",
                   "usr/lib/cuda/bin/x86_64-unknown-linux-gnu-fatbinary")

    target(["apt-get", "clean"])

    target(["ldconfig"])

    tarball = datetime.date.today().strftime(
        f"{os.getcwd()}/%Y-%m-%d-bookworm-amd64-nvidia-rootfs.tar")
    print(tarball)

    subprocess.run([
        "sudo",
        "tar",
        "--exclude=./usr/share/ca-certificates",
        "--exclude=./usr/src",
        "--exclude=./usr/lib/mesa-diverted",
        "--exclude=./usr/bin/X11",
        "--exclude=./usr/lib/systemd/system/system-systemd*cryptsetup.slice",
        "--exclude=./dev",
        "-cf",
        tarball,
        ".",
    ],
                   cwd=ROOTFS_FOLDER,
                   check=True)

    subprocess.run(["sha256sum", tarball], check=True)

    return 0


if __name__ == '__main__':
    sys.exit(main())
