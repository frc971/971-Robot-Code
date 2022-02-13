#!/bin/bash

if [[ "$(hostname)" == "roboRIO"* ]]; then
  /usr/local/natinst/etc/init.d/systemWebServer stop

  ROBOT_CODE="/home/admin/robot_code"

  ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
  ln -s /var/local/natinst/log/FRC_UserProgram.log "${ROBOT_CODE}/FRC_UserProgram.log"
elif [[ "$(hostname)" == "pi-"* ]]; then
  function chrtirq() {
    ps -ef | grep "\\[$1\\]" | awk '{print $2}' | xargs chrt $2 -p $3
  }

  chrtirq "irq/20-fe00b880" -f 50
  chrtirq "irq/66-xhci_hcd" -f 1
  chrtirq "irq/50-VCHIQ do" -o 0
  chrtirq "irq/27-DMA IRQ" -f 50
  chrtirq "irq/51-mmc1" -o 0
  chrtirq "irq/51-mmc0" -o 0
  chrtirq "irq/51-s-mmc0" -o 0
  chrtirq "irq/64-v3d" -o 0
  chrtirq "irq/24-vc4 hvs" -o 0
  chrtirq "irq/42-vc4 hdmi" -o 0
  chrtirq "irq/43-vc4 hdmi" -o 0
  chrtirq "irq/39-vc4 hdmi" -o 0
  chrtirq "irq/39-s-vc4 hd" -o 0
  chrtirq "irq/38-vc4 hdmi" -o 0
  chrtirq "irq/38-s-vc4 hd" -o 0
  chrtirq "irq/29-DMA IRQ" -f 50
  chrtirq "irq/48-vc4 hdmi" -o 0
  chrtirq "irq/49-vc4 hdmi" -o 0
  chrtirq "irq/45-vc4 hdmi" -o 0
  chrtirq "irq/45-s-vc4 hd" -o 0
  chrtirq "irq/44-vc4 hdmi" -o 0
  chrtirq "irq/44-s-vc4 hd" -o 0
  chrtirq "irq/30-DMA IRQ" -f 50
  chrtirq "irq/19-fe004000" -f 50
  chrtirq "irq/34-vc4 crtc" -o 0
  chrtirq "irq/35-vc4 crtc" -o 0
  chrtirq "irq/36-vc4 crtc" -o 0
  chrtirq "irq/35-vc4 crtc" -o 0
  chrtirq "irq/37-vc4 crtc" -o 0
  chrtirq "irq/23-uart-pl0" -o 0
  chrtirq "irq/57-eth0" -f 10
  chrtirq "irq/58-eth0" -f 10

  # We have systemd configured to handle restarting, so just exec.
  export PATH="${PATH}:/home/pi/robot_code"
  rm -rf /dev/shm/aos
  exec starterd
else
  ROBOT_CODE="${HOME}/robot_code"
fi

cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
  rm -rf /dev/shm/aos
  starterd 2>&1
done
