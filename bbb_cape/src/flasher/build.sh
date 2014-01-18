#!/bin/bash

set -e

cd $(dirname $0)

[[ -d stm32flash ]] || ( git clone https://git.gitorious.org/stm32flash/stm32flash.git stm32flash &&
	cd stm32flash && git checkout 8399fbe1baf2b7d097746786458021d92895d71b )

../../../aos/build/build.sh linux flasher.gyp no flasher "$@"
