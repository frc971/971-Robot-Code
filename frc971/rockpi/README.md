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

`sudo dd if=arm64_bullseye_debian.img of=/dev/sda status=progress`

The default user is `pi`, and password is `raspberry`.
