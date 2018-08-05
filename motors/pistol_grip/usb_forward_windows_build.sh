#!/bin/bash

# This script cross-compiles usb_forward for Windows.

set -e
set -u
set -o pipefail

readonly SOURCE_FILE="$1"
readonly LIBUSB_ARCHIVE="$2"
readonly OUTPUT="$3"

readonly TEMP="${OUTPUT}.build"
rm -rf "${TEMP}"
mkdir -p "${TEMP}/libusb"
tar xJf "${LIBUSB_ARCHIVE}" -C "${TEMP}/libusb"

mkdir -p "${TEMP}/compiler_bin"
ln -s $(readlink -f external/mingw_compiler/usr/bin/x86_64-w64-mingw32-as) "${TEMP}/compiler_bin/as"

export LD_LIBRARY_PATH=external/mingw_compiler/usr/lib/x86_64-linux-gnu:external/mingw_compiler/lib/x86_64-linux-gnu:external/mingw_compiler/usr/lib
export PATH=external/mingw_compiler/usr/bin:$PATH

x86_64-w64-mingw32-g++-posix \
	--std=gnu++11 \
	-D__USE_MINGW_ANSI_STDIO \
	-o "${OUTPUT}" \
	-I "${TEMP}/libusb/include" \
	"${SOURCE_FILE}" \
	-static -static-libgcc -static-libstdc++ \
	-lws2_32 \
	-L "${TEMP}/libusb/MinGW64/static" -lusb-1.0 \
	-B"${TEMP}/compiler_bin"
