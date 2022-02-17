# Creating an SD card to run the Raspberry Pis

This modifies a stock debian root filesystem to be able to operate as a vision
pi.  It is not trying to be reproducible, but should be good enough for FRC
purposes.

The default hostname and IP is pi-971-1, 10.9.71.101.
  Username pi, password raspberry.

## Build the real-time kernel using `build_kernel.sh`

- Checkout the real-time kernel source code, e.g.,
  `cd CODE_DIR`
  `git clone git@github.com:frc971/linux.git`
  `git checkout frc971-5.10-pi4-rt branch`

- Run `build_kernel.sh` to compile the real-time kernel
  `cd ROOTFS_DIR` (where ROOTFS_DIR -> //frc971/raspi/rootfs)
  `./build_kernel.sh CODE_DIR/linux kernel_5.10.tar.gz`

## Download the Raspberry Pi OS

Download the appropriate Raspberry Pi OS image, e.g.,
`2022-01-28-raspios-bullseye-arm64-lite.img` (or any newer arm64
bullseye version, as a .zip file) from
`https://www.raspberrypi.org/downloads/raspberry-pi-os/`, and extract
(unzip) the .img file.

## Create our custom OS image using `modify_root.sh`

- Edit `modify_rootfs.sh` to point to the kernel file (e.g.,
`KERNEL=kernel_5.10.tar.gz`) and the Raspberry Pi OS image (e.g.,
`IMAGE=2022-01-28-raspios-bullseye-arm64-lite.img`)

- Run modify_rootfs.sh to build the filesystem (you might need to hit
return in a spot or two and will need sudo privileges to mount the
partition):
  * `./modify_root.sh`

## Write the file system to the SD card

VERY IMPORTANT NOTE: Before doing the next step, use `lsblk` to find
the device and make absolutely sure this isn't your hard drive or
something else.  It will target /dev/sda by default, which in some
computers is your default hard drive.

After confirming the target device, edit the `make_sd.sh` script to point to the correct IMAGE filename, and run the `make_sd.sh` command,
which takes the name of the pi as an argument:
  * `make_sd.sh pi-971-1`

OR, if you want to manually run this, you can deploy the image by
copying the contents of the image to the SD card.  You can do this
manually, via
  `dd if=2020-02-13-raspbian-bullseye-lite-frc-mods.img of=/dev/sdX bs=1M`

From there, transfer the SD card to the pi, log in, `sudo` to `root`,
and run `/root/bin/change_hostname.sh` to change the hostname to the
actual target.


A couple additional notes on setting this up:
   * You'll likely need to install (`sudo apt install`) the emulation packages `proot` and `qemu-user-static` (or possibly `qemu-arm-static`)
   * If the modify_root.sh script fails, you may need to manually unmount the image (`sudo umount ${PARTITION}` and `rmdir ${PARTITION}`) before running it again
   * Don't be clever and try to link to the image file in a different folder.  These scripts need access directly to the file and will fail otherwise


Things to do once the SD card is complete, and you've booted a PI with it:

  * Download the code:
    Once this is completed, you can boot the pi with the newly flashed SD
    card, and download the code to the pi using:
      `bazel run -c opt --config=armv7 //y2022:pi_download_stripped -- PI_IP_ADDR

    where PI_IP_ADDR is the IP address of the target pi, e.g., 10.9.71.101
