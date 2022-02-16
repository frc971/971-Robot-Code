#!/bin/bash

# Helper script to turn LEDS on or off
# pass argument "off" to turn off; otherwise, it turns them on


if [ "$1" == "off" ]; then
    echo "Turning LEDS off"
    if [ -e /sys/class/gpio/gpio13/value ]; then
        echo 0 > /sys/class/gpio/gpio13/value
        echo 13 > /sys/class/gpio/unexport
    fi
    if [ -e /sys/class/gpio/gpio10/value ]; then
        echo 1 > /sys/class/gpio/gpio10/value
        echo 10 > /sys/class/gpio/unexport
    fi
else
    echo "Turning LEDS on full"
    echo 13 > /sys/class/gpio/export
    echo 10 > /sys/class/gpio/export
    echo "out" > /sys/class/gpio/gpio10/direction
    echo "out" > /sys/class/gpio/gpio13/direction
    echo 1 > /sys/class/gpio/gpio13/value
    echo 0 > /sys/class/gpio/gpio10/value
fi
