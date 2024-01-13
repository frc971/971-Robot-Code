#!/usr/bin/sh

# Helper script to spoof joystick state of robot enabled, to triggger image logging

# Currently, this is going through pi6.  Need to set the right IP address for the bot
imu_pi6="pi@10.79.71.106"

# TODO(milind): add logger in the future
echo "Sending Joystick command '$1' to $imu_pi6"
ssh ${imu_pi6} "bin/aos_send /imu/aos aos.JoystickState '{\"enabled\": $1}'"

if [ $1 = "false" ]
then
    # This extra sleep is necessary to make sure the logs rotate to a new file
    sleep 6
    echo "Sending Joystick command '$1' to $imu_pi6"
    ssh ${imu_pi6} "bin/aos_send /imu/aos aos.JoystickState '{\"enabled\": $1}'"
fi
