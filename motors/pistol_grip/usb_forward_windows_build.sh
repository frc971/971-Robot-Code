#!/bin/bash

# This script cross-compiles usb_forward for Windows.

set -e
set -u
set -o pipefail

check_installed() {
	if ! dpkg -l "$1" > /dev/null 2>&1 ; then
		echo "Must install $1" >&2
		exit 1
	fi
}

check_installed g++-mingw-w64-x86-64

readonly SOURCE_FILE="$1"
readonly LIBUSB_ARCHIVE="$2"
readonly OUTPUT="$3"

readonly TEMP="${OUTPUT}.build"
rm -rf "${TEMP}"
mkdir -p "${TEMP}/libusb"
tar xJf "${LIBUSB_ARCHIVE}" -C "${TEMP}/libusb"

x86_64-w64-mingw32-g++-posix \
	--std=gnu++11 \
	-D__USE_MINGW_ANSI_STDIO \
	-o "${OUTPUT}" \
	-I "${TEMP}/libusb/include" \
	"${SOURCE_FILE}" \
	-static -static-libgcc -static-libstdc++ \
	-lws2_32 \
	-L "${TEMP}/libusb/MinGW64/static" -lusb-1.0
