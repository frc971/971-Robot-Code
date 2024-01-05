# Building the rootfs for the Orin NX 8g

Our code targets Debian Bookworm.  Debian support for the Orin is poor, but
yocto support is pretty good.  So, rather than try to recreate what Yocto has,
we instead install the .deb's that Yocto generates for the NVIDIA libraries
and install them on a bookworm sysroot.  To do this:

Check out and follow the instructions on https://github.com/frc971/meta-frc971
to create a working image for the Orin.  You will need this to install.

Then, build a Bookworm image.

```
./build_rootfs.py
```

You'll likely have to point that to your yocto build unless your username is
austin.

From there, copy the resulting image to `./frc971-image-orin-nx-8g.tegraflash.tar.gz`

Then, hold the bootloader switch, reboot the Orin, and then run the install
script to install it on your orin.
```
doflash_frc971.sh
```
