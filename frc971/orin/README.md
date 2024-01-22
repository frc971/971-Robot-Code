# Building the rootfs for the Orin NX 8g

Our code targets Debian Bookworm.  Debian support for the Orin is poor, but
yocto support is pretty good.  So, rather than try to recreate what Yocto has,
we instead install the .deb's that Yocto generates for the NVIDIA libraries
and install them on a bookworm sysroot.  To do this:

Check out and follow the instructions on https://github.com/frc971/meta-frc971
to create a working image for the Orin.  You will need this to install.

Then, build a Bookworm image.  You'll likely have to set the YOCTO variable in build_rootfs.py to point to your yocto build unless your username is austin.

From the 971-Robot-Code/frc971/orin directory:

```
./build_rootfs.py
```

From the yocto folder (it should be under `yocto/build/tmp/deploy/images/orin-nx-8g/frc971-image-orin-nx-8g.tegraflash.tar.gz`), copy the image that was created in the bitbake step to `frc971/orin//frc971-image-orin-nx-8g.tegraflash.tar.gz`

Then, hold the bootloader switch, reboot the Orin (the fan should not
turn on initially), and then run the install script to install it on
your orin.

```
doflash_frc971.sh
```
