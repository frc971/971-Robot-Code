#!/bin/bash

set -eux
set -o pipefail

UBOOT_VERSION=v2022.10

IMAGE="arm64_bullseye_debian.img"
KERNEL_VERSION=6.0.8-rt14-rockpi4b
PARTITION="${IMAGE}.partition"

# Check if dependencies are missing.
missing_deps=()
REQUIRED_DEPS=(
    bison
    debootstrap
    device-tree-compiler
    flex
    gcc-aarch64-linux-gnu
    gcc-arm-none-eabi
    swig
    u-boot-tools
)
for dep in "${REQUIRED_DEPS[@]}"; do
    if ! dpkg-query -W -f='${Status}' "${dep}" | grep -q "install ok installed"; then
        missing_deps+=("${dep}")
    fi
done

# Print missing dependencies
if ((${#missing_deps[@]} != 0 )); then
    echo "Missing dependencies, please install:"
    echo apt install "${missing_deps[@]}"
    exit 1
else
    echo -e "\033[32mAll dependencies are already installed\033[0m"
fi

export CC=aarch64-linux-gnu-

# Reset any existing mounts.
if mount | grep "${PARTITION}/boot" >/dev/null; then
  sudo umount "${PARTITION}/boot"
fi

if mount | grep "${PARTITION}" >/dev/null; then
  sudo umount "${PARTITION}"
fi

LOOPBACK="$(sudo losetup --list | awk "/$IMAGE/"'{print $1}')"
if [[ -n "${LOOPBACK}" ]]; then
  echo "Loop still exists..."
  sudo losetup -d "${LOOPBACK}"
fi

# Build bl31.elf.
if [[ ! -e arm-trusted-firmware ]]; then
  git clone https://github.com/ARM-software/arm-trusted-firmware --depth=1
fi

pushd arm-trusted-firmware/
git pull --ff-only
#make CROSS_COMPILE="${CC}" realclean -j "$(nproc)"
make CROSS_COMPILE="${CC}" PLAT=rk3399 -j "$(nproc)"
popd

export BL31="$(pwd)/arm-trusted-firmware/build/rk3399/release/bl31/bl31.elf"

ls -lah "${BL31}"

# Now, build uboot.
if [[ ! -e u-boot ]]; then
  git clone -b "${UBOOT_VERSION}" https://github.com/u-boot/u-boot --depth=1
fi

pushd u-boot/
git fetch origin "${UBOOT_VERSION}"
git reset --hard "${UBOOT_VERSION}"

#make ARCH=arm CROSS_COMPILE="${CC}" distclean -j $(nproc)
make ARCH=arm CROSS_COMPILE="${CC}" rock-pi-4-rk3399_defconfig -j "$(nproc)"
make ARCH=arm CROSS_COMPILE="${CC}" -j "$(nproc)"
echo "Made uboot"
popd

function target_mkdir() {
  target "install -d -m $2 -o $(echo $1 | sed 's/\.[^.]*//') -g $(echo $1 | sed 's/[^.]*\.//') $3"
}

function copyfile() {
  sudo cp contents/$3 ${PARTITION}/$3
  sudo chmod $2 ${PARTITION}/$3
  target "chown $1 /$3"
}


# Now build the base set of partitions.
NEW_IMAGE=0
if [[ ! -e "${IMAGE}" ]]; then
  NEW_IMAGE=1
  dd if=/dev/zero of="${IMAGE}" bs=1 count=0 seek=3G
  dd if=./u-boot/idbloader.img of="${IMAGE}" seek=64 conv=notrunc
  dd if=./u-boot/u-boot.itb of="${IMAGE}" seek=16384 conv=notrunc

  sfdisk "${IMAGE}" <<-__EOF__
16M,64M,L,*
80M,,L,*
__EOF__

  sudo losetup -P -f "${IMAGE}"
  LOOPBACK="$(sudo losetup --list | grep "${IMAGE}" | awk '{print $1}')"

  sudo mkfs.fat "${LOOPBACK}p1"
  sudo mkfs.ext4 -L rootfs "${LOOPBACK}p2"

  sudo losetup -d "${LOOPBACK}"
fi


# Mount them
mkdir -p "${PARTITION}"

sudo losetup -P -f "${IMAGE}"
LOOPBACK="$(sudo losetup --list | grep "${IMAGE}" | awk '{print $1}')"
sudo mount "${LOOPBACK}p2" "${PARTITION}"
if [[ ! -e "${PARTITION}/boot" ]]; then
  sudo mkdir "${PARTITION}/boot"
fi
sudo mount "${LOOPBACK}p1" "${PARTITION}/boot"

# Run the string command as root inside the target.
function target() {
  sudo chroot --userspec=root:root "${PARTITION}" qemu-aarch64-static \
    /bin/bash -c "$1"
}

# Run the string command as the pi user inside the target.
function pi_target() {
  sudo chroot --userspec=pi:pi --groups=pi "${PARTITION}" qemu-aarch64-static \
    /bin/bash -c "$1"
}

# And, if we made a new image, debootstrap to get things going.
if (( NEW_IMAGE == 1 )); then
  sudo debootstrap --arch=arm64 --no-check-gpg --foreign bullseye \
    "${PARTITION}" http://deb.debian.org/debian/
  sudo cp /usr/bin/qemu-aarch64-static "${PARTITION}/usr/bin/"

  target "/debootstrap/debootstrap --second-stage"

  target "useradd -m -p '\$y\$j9T\$85lzhdky63CTj.two7Zj20\$pVY53UR0VebErMlm8peyrEjmxeiRw/rfXfx..9.xet1' -s /bin/bash pi"

else
  sudo cp /usr/bin/qemu-aarch64-static "${PARTITION}/usr/bin/"
fi

# Install the kernel.
sudo rm -rf "${PARTITION}/boot/dtbs/${KERNEL_VERSION}/"
sudo rm -rf "${PARTITION}/lib/modules/${KERNEL_VERSION}/"
sudo mkdir -p "${PARTITION}/lib/modules/"
sudo tar --owner=0 --group=0 --no-same-owner --no-same-permissions -xvf \
  "linux-kernel-${KERNEL_VERSION}.tar.xz" -C "${PARTITION}/boot/" \
  --strip-components=2 ./boot/
sudo tar --strip-components=3 -xvf "linux-kernel-${KERNEL_VERSION}.tar.xz" \
  -C "${PARTITION}/lib/modules/" ./lib/modules

# Now, configure it to start automatically.
cat << __EOF__ | sudo tee "${PARTITION}/boot/sdcard_extlinux.conf"
label Linux ${KERNEL_VERSION}
    kernel /vmlinuz-${KERNEL_VERSION}
    append earlycon=uart8250,mmio32,0xff1a0000 earlyprintk console=ttyS2,1500000n8 root=/dev/mmcblk0p2 ro rootfstype=ext4 rootflags=data=journal rootwait
    fdtdir /dtbs/${KERNEL_VERSION}/
__EOF__
cat << __EOF__ | sudo tee "${PARTITION}/boot/emmc_extlinux.conf"
label Linux ${KERNEL_VERSION}
    kernel /vmlinuz-${KERNEL_VERSION}
    append earlycon=uart8250,mmio32,0xff1a0000 earlyprintk console=ttyS2,1500000n8 root=/dev/mmcblk1p2 ro rootfstype=ext4 rootflags=data=journal rootwait
    fdtdir /dtbs/${KERNEL_VERSION}/
__EOF__

mkimage -c none -A arm -T script -d contents/boot/boot.script contents/boot/boot.scr
copyfile root.root 644 boot/boot.scr
rm contents/boot/boot.scr
copyfile root.root 644 etc/apt/sources.list.d/bullseye-backports.list
copyfile root.root 644 etc/apt/sources.list.d/frc971.list

target "apt-get update"
target "apt-get -y install -t bullseye-backports systemd systemd-resolved"

# Make systemd-resolved work.
target_mkdir root.root 755 run/systemd
target_mkdir systemd-resolve.systemd-resolve 755 run/systemd/resolve
copyfile systemd-resolve.systemd-resolve 644 run/systemd/resolve/stub-resolv.conf
target "systemctl enable systemd-resolved"

target "apt-get -y install -t bullseye-backports bpfcc-tools"

target "apt-get install -y sudo openssh-server python3 bash-completion git v4l-utils cpufrequtils pmount rsync vim-nox chrony libopencv-calib3d4.5 libopencv-contrib4.5 libopencv-core4.5 libopencv-features2d4.5 libopencv-flann4.5 libopencv-highgui4.5 libopencv-imgcodecs4.5 libopencv-imgproc4.5 libopencv-ml4.5 libopencv-objdetect4.5 libopencv-photo4.5 libopencv-shape4.5 libopencv-stitching4.5 libopencv-superres4.5 libopencv-video4.5 libopencv-videoio4.5 libopencv-videostab4.5 libopencv-viz4.5 libnice10 pmount libnice-dev feh libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 libgstreamer-plugins-bad1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-nice usbutils locales trace-cmd clinfo jq strace sysstat lm-sensors"
target "cd /tmp && wget https://software.frc971.org/Build-Dependencies/libmali-midgard-t86x-r14p0-x11_1.9-1_arm64.deb && sudo dpkg -i libmali-midgard-t86x-r14p0-x11_1.9-1_arm64.deb && rm libmali-midgard-t86x-r14p0-x11_1.9-1_arm64.deb"

target "apt-get clean"

target "usermod -a -G sudo pi"
target "usermod -a -G video pi"
target "localedef -i en_US -f UTF-8 en_US.UTF-8"

copyfile root.root 644 etc/fstab
copyfile root.root 440 etc/sudoers
copyfile root.root 444 etc/default/cpufrequtils
target_mkdir root.root 755 etc/systemd/network
copyfile root.root 644 etc/systemd/network/eth0.network
target_mkdir pi.pi 755 home/pi/.ssh
copyfile pi.pi 600 home/pi/.ssh/authorized_keys
target_mkdir root.root 700 root/bin
copyfile root.root 500 root/bin/grow.sh
copyfile root.root 500 root/bin/change_hostname.sh
copyfile root.root 500 root/bin/deploy_kernel.sh
copyfile root.root 500 root/bin/chrt.sh
copyfile root.root 644 etc/systemd/system/grow-rootfs.service
copyfile root.root 644 etc/systemd/system/mount-boot.service
copyfile root.root 644 etc/sysctl.d/sctp.conf
copyfile root.root 644 etc/systemd/logind.conf
copyfile root.root 555 etc/bash_completion.d/aos_dump_autocomplete
copyfile root.root 444 etc/modules-load.d/adis16505.conf
copyfile root.root 644 etc/security/limits.d/rt.conf
copyfile root.root 644 etc/systemd/system/frc971.service
copyfile root.root 644 etc/systemd/system/frc971chrt.service
copyfile root.root 644 etc/systemd/system/usb-mount@.service
copyfile root.root 644 etc/udev/rules.d/99-usb-mount.rules
copyfile root.root 644 etc/udev/rules.d/99-adis16505.rules
copyfile root.root 644 etc/udev/rules.d/99-mali.rules
copyfile root.root 644 etc/udev/rules.d/99-coral-perms.rules
copyfile root.root 644 etc/chrony/chrony.conf

target "apt-get update"

target "systemctl enable systemd-networkd"
target "systemctl enable grow-rootfs"
target "systemctl enable mount-boot"
target "systemctl enable frc971"
target "systemctl enable frc971chrt"
target "/root/bin/change_hostname.sh pi-971-1"


if [[ ! -e "${PARTITION}/home/pi/.dotfiles" ]]; then
  pi_target "cd /home/pi/ && \
    git clone --separate-git-dir=/home/pi/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && \
    rsync --recursive --verbose --exclude .git tmpdotfiles/ /home/pi/ && \
    rm -r tmpdotfiles && \
    git --git-dir=/home/pi/.dotfiles/ --work-tree=/home/pi/ config --local status.showUntrackedFiles no"
  pi_target "vim -c \":qa!\""

  target "cd /root/ && \
    git clone --separate-git-dir=/root/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && \
    rsync --recursive --verbose --exclude .git tmpdotfiles/ /root/ && rm -r tmpdotfiles && \
    git --git-dir=/root/.dotfiles/ --work-tree=/root/ config --local status.showUntrackedFiles no"
  target "vim -c \":qa!\""
fi

target "apt-get clean"

# Add a file to show when this image was last modified and by whom
TIMESTAMP_FILE="${PARTITION}/home/pi/.ImageModifiedDate.txt"
echo "Date modified:"`date` > "${TIMESTAMP_FILE}"
echo "Image file: ${IMAGE}"  >> "${TIMESTAMP_FILE}"
echo "Git tag: "`git rev-parse HEAD` >> "${TIMESTAMP_FILE}"
echo "User: "`whoami` >> "${TIMESTAMP_FILE}"

sudo chroot ${PARTITION} qemu-aarch64-static /bin/bash

# TODO(austin): This appears to not be working...  pi_target doesn't apper to be happy
sudo chroot --userspec=pi:pi --groups=pi ${PARTITION} qemu-aarch64-static /bin/bash

# TODO(austin): everything else we were doing to the pi's.
sudo rm ${PARTITION}/usr/bin/qemu-aarch64-static
sudo umount ${PARTITION}/boot
sudo umount ${PARTITION}
rmdir ${PARTITION}
