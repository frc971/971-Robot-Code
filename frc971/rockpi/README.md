# Building a root filesystem image

To start with, you need to build the kernel.
`build_rootfs.sh` has a list of dependencies you will need to build everything
in a comment.  Start by installing those.

Then, run `./build_kernel.sh`. This will make a .tar.xz with the kernel and
device tree in it.

Then, build the rootfs with `./build_rootfs.sh`.  This will make an image
named `arm64_bullseye_debian.img`.

The script is set up to reinstall the kernel, add any missing packages, and
add/update the files added.  This isn't perfect, but will incrementally update
a rootfs as we go.  When in doubt, a full reinstall is recommended.
Do that by removing the image.

# Installing

We're currently installing the filesystem onto the eMMC of the Rock
Pis.  This involves a multi-step process:

* Create an SD card with the image.  The Pi will boot off of this in order to flash the eMMC

Assuming the SD card shows up on your system as /dev/sda, then you can use the command:

`sudo dd if=arm64_bullseye_debian.img of=/dev/sda status=progress`

WARNING: any time you're using 'dd' to write data, make SURE you've got the right output
device, e.g., here of=/dev/sda.  It's possible your hard drive is /dev/sda, and you
don't want to over-write that!!

* Create a USB drive with the image on it.  We'll use this to copy the image to the eMMC

We're using the xfs filesystem for USB thumb drives.  To create this, do the following:
* Insert the thumbdrive
* Figure out which device this is and unmount it.  E.g., if it's /dev/sdb1, do
  * `sudo umount /dev/sdb1`
* Run fdisk to create an empty partition:
  * `sudo fdisk /dev/sdb`
  * Type "o" to create an empty DOS partition table
  * Type "n" to create a new partition
  * Type "p" to select primary partition
  * Accept the next 3 default values (1, 2048, 500170751) (or whatever end block is given)
  * Type "w" to write this information to the partition table
* Run mkfs to make this an xfs file system:
  * `sudo mkfs -t xfs -f /dev/sdb1`
    * (Note, you might have to install the xfs file system support for mkfs)
* Change permissions on the drive to make it write-able
  * `sudo chmod a+rwx [path-to-drive]`
    * where `[path-to-drive]` is the path to the drive, e.g., /media/jim/XXXX
    (I believe /dev/sdb1 should work, also)

# Writing the disk image to the eMMC

Insert the SD card and the thumbdrive and boot the Pi.

It should come up as pi-971-1 (IP: 10.9.71.101).  The default user is
`pi`, and password is `raspberry`.  Note: if you don't have your ssh
key installed, you should get that added to the base image.

Once booted, login and write the image to the eMMC:

`sudo dd if=/media/sda1/arm64_bullseye_debian.img of=/dev/mmcblk1 status=progress`

Then, power down the Pi, remove the SD card and the USB thumb drive, and power on the Pi

Once it's up, login again.

Change the hostname to the desired hostname, e.g., if you want it to be pi-9971-2, use

`sudo /root/bin/change_hostname.sh pi-9971-2`

It's a good idea here to reboot the Pi one more time to make sure it
came up with the expected IP address and hostname.

# Additional steps in setting up the Rock Pis

We also have been plugging in a USB thumb drive to each Pi to do logging.

We are formatting these as xfs filesystems.  To do this follow the
above instructions on a new USB drive to format as xfs.

That's it!!


# Other notes:

# State of RK3399 image processing.

Use `media-ctl -p -d /dev/media1` to print out the ISP configuration.

There is configuration in the image sensor driver, ISP, and resizer.
`media-ctl` can be used for all of that, and everything needs to be setup
before images will flow.  The resizer supports exporting 2 image sizes
simultaneously, which is incredibly useful to feed both the h264 encoder and
code with different image sizes.  The following captures images:

```
# set the links
media-ctl -v -d "platform:rkisp1" -r
media-ctl -v -d "platform:rkisp1" -l "'ov5647 4-0036':0 -> 'rkisp1_csi':0 [1]"

media-ctl -v -d "platform:rkisp1" -l "'rkisp1_csi':1 -> 'rkisp1_isp':0 [1]"
media-ctl -v -d "platform:rkisp1" -l "'rkisp1_isp':2 -> 'rkisp1_resizer_selfpath':0 [1]"
media-ctl -v -d "platform:rkisp1" -l "'rkisp1_isp':2 -> 'rkisp1_resizer_mainpath':0 [1]"

# set format for imx219 4-0010:0
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"ov5647 4-0036":0 [fmt:SBGGR10_1X10/1296x972]'

# set format for rkisp1_isp pads:
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_isp":0 [fmt:SBGGR10_1X10/1296x972 crop: (0,0)/1296x972]'
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_isp":2 [fmt:YUYV8_2X8/1296x972 crop: (0,0)/1296x972]'

# set format for rkisp1_resizer_selfpath pads:
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_resizer_selfpath":0 [fmt:YUYV8_2X8/1296x972 crop: (0,0)/1296x972]'
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_resizer_selfpath":1 [fmt:YUYV8_2X8/1296x972]'

media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_resizer_mainpath":0 [fmt:YUYV8_2X8/1296x972 crop: (0,0)/1296x972]'
media-ctl -v -d "platform:rkisp1" --set-v4l2 '"rkisp1_resizer_mainpath":1 [fmt:YUYV8_2X8/648x486]'

# set format for rkisp1_selfpath:
v4l2-ctl -z "platform:rkisp1" -d "rkisp1_selfpath" -v "width=1296,height=972,"
v4l2-ctl -z "platform:rkisp1" -d "rkisp1_selfpath" -v "pixelformat=422P"

v4l2-ctl -z "platform:rkisp1" -d "rkisp1_mainpath" -v "width=648,height=486,"
v4l2-ctl -z "platform:rkisp1" -d "rkisp1_mainpath" -v "pixelformat=422P"

# start streaming:
echo "Selfpath"
v4l2-ctl -z "platform:rkisp1" -d "rkisp1_selfpath" --stream-mmap --stream-count 10

echo "Mainpath"
v4l2-ctl -z "platform:rkisp1" -d "rkisp1_mainpath" --stream-mmap --stream-count 10
```

There are 2 pieces of hardware in the rockpi for encoding/decoding,
the Hantro encoder/decoder, and the rkvdec decoder.
Considering all the robot does is encode, we don't really need to worry about
rkvdec.

ISP configuration is available
[here](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-meta-rkisp1.html#c.rkisp1_params_cfg)
using the v4l2 API.

[CNX software](https://www.cnx-software.com/2020/11/24/hantro-h1-hardware-accelerated-video-encoding-support-in-mainline-linux/)
has a decent state of the union from 2020.  The names and pointers are still relevant

July 2022 has some patches which suggest they might make h264 encoding work.
I don't know where this repo comes from though.
[Potential patches](https://git.pengutronix.de/cgit/mgr/linux/log/?h=v5.19/topic/rk3568-vepu-h264-stateless-bootlin)

[mpp](https://github.com/rockchip-linux/mpp/blob/develop/readme.txt)
is Rockchip's proposed userspace processing library.  I haven't gotten this to
work with 6.0 yet.

gstreamer only shows codecs in `gst-inspect-1.0 video4linux2` which there are
encoders for.

https://lwn.net/Articles/776082/ has a reference patch for the vendor driver.

In theory, something like:
`gst-launch-1.0 -vvvv videotestsrc ! v4l2jpegenc ! fakesink`
should work for using the m2m kernel implementation of JPEG encoding on the
hantro encoder, but something isn't happy and doesn't work.  I need to do more debugging
to figure that out, if we care.  It feels like a good baby step to a h624 encoder,
I could be wrong there.

[This](https://lkml.org/lkml/2021/11/16/628) adds support for VP9 decoding, but
our kernel already has it.  It has pointers to the pieces which added decoding.

[This](https://www.netbsd.org/~mrg/rk3399/Rockchip%20RK3399TRM%20V1.1%20Part3%2020160728.pdf)
is the TRM for the chip.
