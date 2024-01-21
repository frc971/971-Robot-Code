#!/bin/bash

set -ex

chmod a+rwx /sys/kernel/debug

# Make sure /sys/kernel/tracing is reasonably accessible so --enable_ftrace
# works.
chmod a+w /sys/kernel/tracing/trace_marker
echo 10000 > /sys/kernel/tracing/buffer_size_kb
chmod a+rw /sys/kernel/tracing/tracing_on
chmod a+rwx /sys/kernel/tracing
