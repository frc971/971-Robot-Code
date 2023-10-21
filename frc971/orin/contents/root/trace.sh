#!/bin/sh

echo 1 > /sys/kernel/debug/tracing/tracing_on
echo 30720 > /sys/kernel/debug/tracing/buffer_size_kb
echo 1 > /sys/kernel/debug/tracing/events/tegra_rtcpu/enable
echo 1 > /sys/kernel/debug/tracing/events/freertos/enable
echo 2 > /sys/kernel/debug/camrtc/log-level
echo 1 > /sys/kernel/debug/tracing/events/camera_common/enable
echo > /sys/kernel/debug/tracing/trace

echo file vi2_fops.c +p > /sys/kernel/debug/dynamic_debug/control
echo file csi2_fops.c +p > /sys/kernel/debug/dynamic_debug/control

echo file vi4_fops.c +p > /sys/kernel/debug/dynamic_debug/control
echo file csi.c +p > /sys/kernel/debug/dynamic_debug/control
echo file csi4_fops.c +p > /sys/kernel/debug/dynamic_debug/control
echo file nvcsi.c +p > /sys/kernel/debug/dynamic_debug/control

cat /sys/kernel/debug/tracing/trace /sys/kernel/debug/tracing/trace_pipe
