#!/usr/bin/python3

import argparse, os, logging, tempfile, subprocess, datetime
from buildfile_lib import generate_build_file

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=os.environ.get('LOGLEVEL', 'INFO').upper(),
    format="%(name)s [%(filename)s:%(lineno)d] %(levelname)s: %(message)s")

filename = f"{str(datetime.date.today())}-arm64-yocto-orin-sysroot"


class MountedLoopback:

    def __init__(self, filename: str, dir: str) -> None:
        self.filename = filename
        self.dir = dir

    def __enter__(self) -> None:
        logger.info(f"Mounting {self.filename} to {self.dir}")
        subprocess.run(["mount", "-o", "loop", self.filename, self.dir],
                       check=True)

    def __exit__(self, _, __, ___) -> None:
        logger.info(f"Unmounting {self.dir}")
        subprocess.run(["umount", self.dir], check=True)


class Loopback:

    def __init__(self, filename: str) -> None:
        self.filename = filename

    def __enter__(self) -> str:
        print(f"Mounting {self.filename} as loopback")
        result = subprocess.run(["losetup", "--show", "-f", self.filename],
                                check=True,
                                stdout=subprocess.PIPE)
        device = result.stdout.decode('utf-8').strip()
        print(f"Mounted {self.filename} to {device}")
        self.device = device
        return device

    def __exit__(self, _, __, ___) -> None:
        print(f"Unmounting {self.device}")
        subprocess.run(["sudo", "losetup", "-d", self.device], check=True)


def build_xfs_image(ext4_image):
    subprocess.run([
        "dd", "if=/dev/zero", f"of={filename}.rootfs", "bs=1", "count=0",
        "seek=8589934592"
    ])

    with Loopback(f"{filename}.rootfs") as device:
        subprocess.run(["ls", "-l", device])
        subprocess.run([
            "sudo", "mkfs.xfs", "-f", "-d", "su=128k", "-d", "sw=1", "-i",
            "nrext64=0", "-L", "rootfs", device
        ])
    with tempfile.TemporaryDirectory() as mntdir, MountedLoopback(
            f"{filename}.rootfs", mntdir):
        subprocess.run(
            ["rsync", "-av", "--progress", f"{ext4_image}/", mntdir])


def generate_rootfs(ext4_image):
    subprocess.run([
        "tar", "--verbose", "--zstd", "-cf", f"{filename}.tar.zst", "-C",
        ext4_image, "./"
    ])


def main():
    parser = argparse.ArgumentParser(
        prog='image_util.py',
        description=
        'Utility program for orin images, by default outputs the xfs image for flashing. Must be run as root so it can mount the package.'
    )

    subparsers = parser.add_subparsers(help='Parsers', dest='command')
    flash_parser = subparsers.add_parser('flash',
                                         help='do help flash to get help')

    flash_parser.add_argument('package',
                              help='The path to the yocto flash package.',
                              type=str)

    flash_parser.add_argument(
        '--rootfs',
        help='The path to the rootfs, should be the generated xfs filesystem.',
        type=str)

    convert_parser = subparsers.add_parser(
        'convert',
        help='Converts an image into xfs NOTE: shouldn\'t be used right now')

    convert_parser.add_argument('package',
                                help='The path to the yocto flash package.',
                                type=str)

    generate_build_file_parser = subparsers.add_parser(
        'generate_build_file',
        help='Generates a build file for the orin image.')
    generate_build_file_parser.add_argument(
        'package', help='The path to the yocto flash package.', type=str)

    generate_rootfs_parser = subparsers.add_parser(
        'generate_rootfs', help='Generates a rootfs file for bazel.')
    generate_rootfs_parser.add_argument(
        'package', help='The path to the yocto flash package.', type=str)

    args = parser.parse_args()

    if os.getuid() != 0:
        print("Please run as root.")
        return

    if args.command == "flash":
        package = args.package
        rootfs = args.rootfs
        with tempfile.TemporaryDirectory() as pkg_dir:
            logger.info(f"Extracting {args.package} into {pkg_dir}.")
            subprocess.run([
                "tar", "-xf", args.package, "--directory", pkg_dir, "--verbose"
            ])
            logger.info(f"Finished extracting.")

            if args.rootfs:
                subprocess.run(["rm", f"{pkg_dir}/frc971-image.ext4"])
                subprocess.run(["rsync", "--progress", args.rootfs, pkg_dir])
                subprocess.run([
                    "mv", f"{pkg_dir}/{rootfs}", f"{pkg_dir}/frc971-image.ext4"
                ])

            wd = os.getcwd()
            logger.info("Starting flashing sequence.")
            os.chdir(pkg_dir)
            subprocess.run(["sudo", "./initrd-flash"])
            os.chdir(wd)
        return
    elif args.command == "convert" or args.command == "generate_build_file" or args.command == "generate_rootfs":
        package = args.package
        with (tempfile.TemporaryDirectory() as
              pkg_dir, tempfile.TemporaryDirectory() as image_mntdir):

            logger.info(f"Extracting {package} into {pkg_dir}.")
            subprocess.run(
                ["tar", "-xf", package, "--directory", pkg_dir, "--verbose"])
            logger.info(f"Finished extracting.")

            with MountedLoopback(f"{pkg_dir}/frc971-image.ext4", image_mntdir):
                if args.command == "convert":
                    build_xfs_image(image_mntdir)
                elif args.command == "generate_build_file":
                    generate_build_file(image_mntdir)
                else:
                    generate_rootfs(image_mntdir)


if __name__ == '__main__':
    main()
