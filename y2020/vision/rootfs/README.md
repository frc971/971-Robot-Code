This modifies a stock debian root filesystem to be able to operate as a vision
pi.  It is not trying to be reproducible, but should be good enough for FRC
purposes.

The default hostname and IP is pi-8971-1, 10.89.71.101.
  Username pi, password raspberry.

Download 2020-02-13-raspbian-buster-lite.img (or any newer buster version)
from `https://www.raspberrypi.org/downloads/raspberry-pi-os/` and
edit `modify_rootfs.sh` to point to it.  Run modify_rootfs.sh to build the
filesystem (you might need to hit return in a spot or two).

After confirming the target device, deploy by copying the contents of the image
to the SD card.
  `dd if=2020-02-13-raspbian-buster-lite.img of=/dev/sdX bs=1M`

Use `lsblk` to find the device and make absolutely sure this isn't your hard
drive or something else.

From there, log in, `sudo` to `root`, and run `/root/bin/change_hostname.sh` to
change the hostname to the actual target.
