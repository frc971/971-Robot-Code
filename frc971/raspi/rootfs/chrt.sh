#!/bin/bash

set -e

function chrtirq() {
  PIDS="$(ps -ef | grep "\\[$1\\]" | awk '{print $2}')"

  for PID in $PIDS; do
    chrt $2 -p $3 "${PID}"

    ps -q "${PID}" -o comm= | tr -d '[:space:]'
    echo -n " "
    chrt -p "${PID}"
  done

  if [ -z "${PID}" ]; then
    echo "No such IRQ ${1}"
  fi
}

chrtirq "irq/[0-9]*-fe00b880" -f 50
chrtirq "irq/[0-9]*-fe204000" -f 60
chrtirq "irq/[0-9]*-adis1650" -f 61
chrtirq "irq/[0-9]*-xhci_hcd" -f 1
chrtirq "irq/[0-9]*-VCHIQ do" -o 0
chrtirq "irq/[0-9]*-DMA IRQ" -f 50
chrtirq "irq/[0-9]*-mmc1" -o 0
chrtirq "irq/[0-9]*-mmc0" -o 0
chrtirq "irq/[0-9]*-s-mmc0" -o 0
chrtirq "irq/[0-9]*-v3d" -o 0
chrtirq "irq/24-vc4 hvs" -o 0
chrtirq "irq/[0-9]*-vc4 hdmi" -o 0
chrtirq "irq/[0-9]*-s-vc4 hd" -o 0
chrtirq "irq/19-fe004000" -f 50
chrtirq "irq/[0-9]*-vc4 crtc" -o 0
chrtirq "irq/23-uart-pl0" -o 0
chrtirq "irq/[0-9]*-eth0" -f 10

# Route data-ready interrupts to the second core
SPI_IRQ="$(cat /proc/interrupts | grep fe204000.spi | awk '{print $1}' | grep '[0-9]*' -o)"
echo 2 > /proc/irq/"${SPI_IRQ}"/smp_affinity
