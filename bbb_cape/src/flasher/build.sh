#!/bin/bash

set -e

cd $(dirname $0)

[[ -d stm32flash ]] || ( git clone https://git.gitorious.org/stm32flash/stm32flash.git stm32flash &&
	cd stm32flash && git checkout 16fbfe6e5854dc36f41712f60b2282cde7571afd && patch -p1 < ../0001-actually-calculate-and-send-a-checksum-for-individua.patch )

../../../aos/build/build.sh atom flasher.gyp no "$@"
