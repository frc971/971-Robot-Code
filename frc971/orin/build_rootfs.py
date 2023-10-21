#!/usr/bin/python3

import contextlib
import pathlib
import collections
import subprocess
import shlex
import os

IMAGE = "arm64_bookworm_debian_yocto.img"
YOCTO = "/home/austin/local/jetpack/robot-yocto/build"

REQUIRED_DEPS = ["debootstrap", "u-boot-tools"]


@contextlib.contextmanager
def scoped_loopback(image):
    """Mounts an image as a loop back device."""
    result = subprocess.run(["sudo", "losetup", "--show", "-f", image],
                            check=True,
                            stdout=subprocess.PIPE)
    device = result.stdout.decode('utf-8').strip()
    print("Mounted", image, "to", repr(device))
    try:
        yield device
    finally:
        subprocess.run(["sudo", "losetup", "-d", device], check=True)


@contextlib.contextmanager
def scoped_mount(image):
    """Mounts an image as a partition."""
    partition = f"{image}.partition"
    try:
        os.mkdir(partition)
    except FileExistsError:
        pass

    result = subprocess.run(["sudo", "mount", "-o", "loop", image, partition],
                            check=True)

    try:
        yield partition
    finally:
        subprocess.run(
            ["sudo", "rm", f"{partition}/usr/bin/qemu-aarch64-static"])
        subprocess.run(["sudo", "umount", partition], check=True)


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


def make_image(image):
    """Makes an image and creates an xfs filesystem on it."""
    result = subprocess.run([
        "dd", "if=/dev/zero", f"of={image}", "bs=1", "count=0",
        "seek=8589934592"
    ],
                            check=True)

    with scoped_loopback(image) as loopback:
        subprocess.run([
            "sudo", "mkfs.xfs", "-d", "su=128k", "-d", "sw=1", "-L", "rootfs",
            loopback
        ],
                       check=True)


def target_unescaped(cmd):
    """Runs a command as root with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=0:0", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c", cmd
    ],
                   check=True)


def target(cmd):
    """Runs a command as root with escaping."""
    target_unescaped(shlex.join([shlex.quote(c) for c in cmd]))


def pi_target_unescaped(cmd):
    """Runs a command as pi with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=pi:pi", "--groups=pi", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c", cmd
    ],
                   check=True)


def pi_target(cmd):
    """Runs a command as pi with escaping."""
    pi_target_unescaped(shlex.join([shlex.quote(c) for c in cmd]))


def copyfile(owner, permissions, file):
    """Copies a file from contents/{file} with the provided owner and permissions."""
    print("copyfile", owner, permissions, file)
    subprocess.run(["sudo", "cp", f"contents/{file}", f"{PARTITION}/{file}"],
                   check=True)
    subprocess.run(["sudo", "chmod", permissions, f"{PARTITION}/{file}"],
                   check=True)
    target(["chown", owner, f"/{file}"])


def target_mkdir(owner_group, permissions, folder):
    """Creates a directory recursively with the provided permissions and ownership."""
    print("target_mkdir", owner_group, permissions, folder)
    owner, group = owner_group.split('.')
    target(
        ["install", "-d", "-m", permissions, "-o", owner, "-g", group, folder])


def list_packages():
    """Lists all installed packages.

    Returns:
      A dictionary with keys as packages, and values as versions.
    """
    result = subprocess.run([
        "sudo", "chroot", "--userspec=0:0", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c",
        "dpkg-query -W -f='${Package} ${Version}\n'"
    ],
                            check=True,
                            stdout=subprocess.PIPE)

    device = result.stdout.decode('utf-8').strip()

    r = {}
    for line in result.stdout.decode('utf-8').strip().split('\n'):
        package, version = line.split(' ')
        r[package] = version

    return r


def list_yocto_packages():
    """Lists all packages in the Yocto folder.

    Returns:
      list of Package classes.
    """
    Package = collections.namedtuple(
        'Package', ['path', 'name', 'version', 'architecture'])
    result = []
    pathlist = pathlib.Path(f"{YOCTO}/tmp/deploy/deb").glob('**/*.deb')
    for path in pathlist:
        # Strip off the path, .deb, and split on _ to parse the package info.
        s = os.path.basename(str(path))[:-4].split('_')
        result.append(Package(str(path), s[0], s[1], s[2]))

    return result


def install_packages(new_packages, existing_packages):
    """Installs the provided yocto packages, if they are new."""
    # To install the yocto packages, first copy them into a folder in /tmp, then install them, then clean the folder up.
    target(["mkdir", "-p", "/tmp/yocto_packages"])
    try:
        to_install = []
        for package in new_packages:
            if package.name in existing_packages and existing_packages[
                    package.name] == package.version:
                print('Skipping', package)
                continue

            subprocess.run([
                "sudo", "cp", package.path,
                f"{PARTITION}/tmp/yocto_packages/{os.path.basename(package.path)}"
            ],
                           check=True)
            to_install.append(package)

        if len(to_install) > 0:
            target(["dpkg", "-i"] + [
                f"/tmp/yocto_packages/{os.path.basename(package.path)}"
                for package in to_install
            ])

    finally:
        target(["rm", "-rf", "/tmp/yocto_packages"])


def install_virtual_packages(virtual_packages):
    """Builds and installs the provided virtual packages."""
    try:
        target(["mkdir", "-p", "/tmp/yocto_packages"])
        for virtual_package in virtual_packages:
            subprocess.run(
                ["dpkg-deb", "--build", f"virtual_packages/{virtual_package}"],
                check=True)
            subprocess.run([
                "sudo", "cp", f"virtual_packages/{virtual_package}.deb",
                f"{PARTITION}/tmp/yocto_packages/{virtual_package}.deb"
            ],
                           check=True)

        target(["dpkg", "-i"] + [
            f"/tmp/yocto_packages/{package}.deb"
            for package in virtual_packages
        ])

    finally:
        target(["rm", "-rf", "/tmp/yocto_packages"])


def main():
    check_required_deps(REQUIRED_DEPS)

    new_image = not os.path.exists(IMAGE)
    if new_image:
        make_image(IMAGE)

    with scoped_mount(IMAGE) as partition:
        if new_image:
            subprocess.run([
                "sudo", "debootstrap", "--arch=arm64", "--no-check-gpg",
                "--foreign", "bookworm", partition,
                "http://deb.debian.org/debian/"
            ],
                           check=True)

        subprocess.run([
            "sudo", "cp", "/usr/bin/qemu-aarch64-static",
            f"{partition}/usr/bin/"
        ],
                       check=True)

        global PARTITION
        PARTITION = partition

        if new_image:
            target(["/debootstrap/debootstrap", "--second-stage"])

            target([
                "useradd", "-m", "-p",
                '$y$j9T$85lzhdky63CTj.two7Zj20$pVY53UR0VebErMlm8peyrEjmxeiRw/rfXfx..9.xet1',
                '-s', '/bin/bash', 'pi'
            ])
            target(["addgroup", "debug"])
            target(["addgroup", "crypto"])
            target(["addgroup", "trusty"])

        if not os.path.exists(
                f"{partition}/etc/apt/sources.list.d/bullseye-backports.list"):
            copyfile("root.root", "644",
                     "etc/apt/sources.list.d/bullseye-backports.list")
            target(["apt-get", "update"])

        target([
            "apt-get", "-y", "install", "gnupg", "wget", "systemd",
            "systemd-resolved", "locales"
        ])

        target(["localedef", "-i", "en_US", "-f", "UTF-8", "en_US.UTF-8"])

        target_mkdir("root.root", "755", "run/systemd")
        target_mkdir("systemd-resolve.systemd-resolve", "755",
                     "run/systemd/resolve")
        copyfile("systemd-resolve.systemd-resolve", "644",
                 "run/systemd/resolve/stub-resolv.conf")
        target(["systemctl", "enable", "systemd-resolved"])

        target([
            "apt-get", "-y", "install", "bpfcc-tools", "sudo",
            "openssh-server", "python3", "bash-completion", "git", "v4l-utils",
            "cpufrequtils", "pmount", "rsync", "vim-nox", "chrony",
            "libopencv-calib3d406", "libopencv-contrib406",
            "libopencv-core406", "libopencv-features2d406",
            "libopencv-flann406", "libopencv-highgui406",
            "libopencv-imgcodecs406", "libopencv-imgproc406",
            "libopencv-ml406", "libopencv-objdetect406", "libopencv-photo406",
            "libopencv-shape406", "libopencv-stitching406",
            "libopencv-superres406", "libopencv-video406",
            "libopencv-videoio406", "libopencv-videostab406",
            "libopencv-viz406", "libnice10", "pmount", "libnice-dev", "feh",
            "libgstreamer1.0-0", "libgstreamer-plugins-base1.0-0",
            "libgstreamer-plugins-bad1.0-0", "gstreamer1.0-plugins-base",
            "gstreamer1.0-plugins-good", "gstreamer1.0-plugins-bad",
            "gstreamer1.0-plugins-ugly", "gstreamer1.0-nice", "usbutils",
            "locales", "trace-cmd", "clinfo", "jq", "strace", "sysstat",
            "lm-sensors", "can-utils", "xfsprogs", "gstreamer1.0-tools",
            "bridge-utils", "net-tools", "apt-file", "parted", "xxd"
        ])
        target(["apt-get", "clean"])

        target(["usermod", "-a", "-G", "sudo", "pi"])
        target(["usermod", "-a", "-G", "video", "pi"])
        target(["usermod", "-a", "-G", "systemd-journal", "pi"])
        target(["usermod", "-a", "-G", "dialout", "pi"])

        virtual_packages = [
            'libglib-2.0-0', 'libglvnd', 'libgtk-3-0', 'libxcb-glx', 'wayland'
        ]

        install_virtual_packages(virtual_packages)

        yocto_package_names = [
            'tegra-argus-daemon', 'tegra-firmware', 'tegra-firmware-tegra234',
            'tegra-firmware-vic', 'tegra-firmware-xusb',
            'tegra-libraries-argus-daemon-base', 'tegra-libraries-camera',
            'tegra-libraries-core', 'tegra-libraries-cuda',
            'tegra-libraries-eglcore', 'tegra-libraries-glescore',
            'tegra-libraries-glxcore', 'tegra-libraries-multimedia',
            'tegra-libraries-multimedia-utils',
            'tegra-libraries-multimedia-v4l', 'tegra-libraries-nvsci',
            'tegra-libraries-vulkan', 'tegra-nvphs', 'tegra-nvphs-base',
            'libnvidia-egl-wayland1'
        ]
        yocto_packages = list_yocto_packages()
        packages = list_packages()

        install_packages([
            package for package in yocto_packages
            if package.name in yocto_package_names
        ], packages)

        # Now, install the kernel and modules after all the normal packages are in.
        yocto_packages_to_install = [
            package for package in yocto_packages
            if (package.name.startswith('kernel-module-') or package.name.
                startswith('kernel-5.10') or package.name == 'kernel-modules')
        ]

        packages_to_remove = []

        # Remove kernel-module-* packages + kernel- package.
        for key in packages:
            if key.startswith('kernel-module') or key.startswith(
                    'kernel-5.10'):
                already_installed = False
                for index, yocto_package in enumerate(
                        yocto_packages_to_install):
                    if key == yocto_package.name and packages[
                            key] == yocto_package.version:
                        print('Found already installed package', key,
                              yocto_package)
                        already_installed = True
                        del yocto_packages_to_install[index]
                        break
                if not already_installed:
                    packages_to_remove.append(key)

        print("Removing", packages_to_remove)
        if len(packages_to_remove) > 0:
            target(['dpkg', '--purge'] + packages_to_remove)
        print("Installing",
              [package.name for package in yocto_packages_to_install])

        install_packages(yocto_packages_to_install, packages)

        target(["systemctl", "enable", "nvargus-daemon.service"])

        copyfile("root.root", "644", "etc/sysctl.d/sctp.conf")
        copyfile("root.root", "644", "etc/systemd/logind.conf")
        copyfile("root.root", "555",
                 "etc/bash_completion.d/aos_dump_autocomplete")
        copyfile("root.root", "644", "etc/security/limits.d/rt.conf")
        copyfile("root.root", "644", "etc/systemd/system/usb-mount@.service")
        copyfile("root.root", "644", "etc/chrony/chrony.conf")
        target_mkdir("root.root", "700", "root/bin")
        target_mkdir("pi.pi", "755", "home/pi/.ssh")
        copyfile("pi.pi", "600", "home/pi/.ssh/authorized_keys")
        target_mkdir("root.root", "700", "root/bin")
        copyfile("root.root", "644", "etc/systemd/system/grow-rootfs.service")
        copyfile("root.root", "500", "root/bin/change_hostname.sh")
        copyfile("root.root", "700", "root/trace.sh")
        copyfile("root.root", "440", "etc/sudoers")
        copyfile("root.root", "644", "etc/fstab")
        copyfile("root.root", "644",
                 "var/nvidia/nvcam/settings/camera_overrides.isp")

        target_mkdir("root.root", "755", "etc/systemd/network")
        copyfile("root.root", "644", "etc/systemd/network/eth0.network")
        copyfile("root.root", "644", "etc/systemd/network/80-can.network")
        target(["/root/bin/change_hostname.sh", "pi-971-1"])

        target(["systemctl", "enable", "systemd-networkd"])
        target(["systemctl", "enable", "grow-rootfs"])
        target(["apt-file", "update"])

        target(["ldconfig"])

        if not os.path.exists(f"{partition}/home/pi/.dotfiles"):
            pi_target_unescaped(
                "cd /home/pi/ && git clone --separate-git-dir=/home/pi/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && rsync --recursive --verbose --exclude .git tmpdotfiles/ /home/pi/ && rm -r tmpdotfiles && git --git-dir=/home/pi/.dotfiles/ --work-tree=/home/pi/ config --local status.showUntrackedFiles no"
            )
            pi_target(["vim", "-c", "\":qa!\""])

            target_unescaped(
                "cd /root/ && git clone --separate-git-dir=/root/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && rsync --recursive --verbose --exclude .git tmpdotfiles/ /root/ && rm -r tmpdotfiles && git --git-dir=/root/.dotfiles/ --work-tree=/root/ config --local status.showUntrackedFiles no"
            )
            target(["vim", "-c", "\":qa!\""])


if __name__ == '__main__':
    main()
