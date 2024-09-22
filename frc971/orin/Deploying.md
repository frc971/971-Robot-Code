# Deploying

Before following these steps you should follow the steps in [meta-frc971](https://github.com/frc971/meta-frc971).

Once you have the image built you can flash an orin.

If you built it on the build server you'll need to copy it to your local machine,
it should live in `yocto/build/tmp/deploy/images/orin-nx-8g/frc971-image-orin-nx-8g.tegraflash.tar.gz`

To flash you'll need a USB data cable connected from your laptop to the USB-C port on the orin.

Once you have that connected you can enter recovery mode by holding the recovery(top button) and pressing the reset(buttom button).
You should only need to hold it for a few seconds.
Once that's done on linux you should be able to do `lsusb` and see the Nvidia Orin as a usb device

Now that you have it as a USB device you can flash the orin by doing `sudo ./image_util.py flash /path/to/frc971-image-orin-nx-8g.tegraflash.tar.gz`.
