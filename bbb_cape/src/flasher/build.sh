#!/bin/bash

set -e

cd $(dirname $0)

[[ -d stm32flash ]] || ( git clone https://git.gitorious.org/stm32flash/stm32flash.git stm32flash &&
	cd stm32flash && git checkout 5b0e391c539e906df7b97f0b457875d90883ea8e && patch -p1 < ../0001-fixed-the-page-by-page-erase-logic.patch )

# TODO(brians): This breaks the build of the main code. Figure out something
# better for this stuff.

../../../aos/build/build.sh atom flasher.gyp no "$@"
