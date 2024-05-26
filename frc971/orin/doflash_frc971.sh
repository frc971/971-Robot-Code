#!/bin/bash
set -ex

# If we pass in a folder, then use that as TMPDIR and don't clean it up
if [[ -d $1 ]]; then
    TMPDIR=$1
else
    TMPDIR=$(mktemp -d /tmp/yoctoflash.XXXXXXXXXX)
    trap finish EXIT
fi

# Cleanup on exit.
function finish {
  sudo rm -rf "${TMPDIR}"
}

# Call sudo to get it started here, rather than waiting for after tar/cp's
sudo echo "Flashing orin"

# If the files haven't been extracted yet, do so now; otherwise, just
# re-use what's already there
if [[ ! -f ${TMPDIR}/initrd-flash ]]; then 
    # Assumes that the image has been copied into ./
    tar xf frc971-image-orin-nx-8g.tegraflash.tar.gz -C "${TMPDIR}"

    # Replace the rootfs with our new image.
    cp --sparse=always arm64_bookworm_debian_yocto.img "${TMPDIR}/frc971-image.ext4"
fi

cd ${TMPDIR}
sudo ./initrd-flash
