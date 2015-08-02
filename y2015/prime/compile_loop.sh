#!/bin/bash

# Runs `build.sh all` and then waits for a file to be modified in a loop.
# Useful for making changes to the code while continuously making sure they
# compile.
# Requires the util-linux and inotify-tools packages.

chrt -i -p 0 $$
ionice -c 3 -p $$

while true; do
	$(dirname $0)/build.sh all
	echo 'compile_loop.sh: Waiting for a file modification...' 1>&2
	inotifywait -e close_write -r aos frc971 bbb_cape
	echo 'compile_loop.sh: Done waiting for a file modification' 1>&2
done
