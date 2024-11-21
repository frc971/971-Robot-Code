# Notes on troubleshooting network setup

If the roboRIO has been configued to use a static IP address like
10.9.71.2, set the laptop to have an IP address on the 10.9.71.x
subnet with a netmask of 255.0.0.0.  The ".x" is different than the
.2 for the roboRIO or any other device on the network.  The driver
station uses .5 or .6 so avoid those.  The radio uses .1 or .50 so
avoid those too.  A good choice might be in the 90-99 range.  If you
are at the school, disconnect from the student wireless network or
try setting your netmask to 255.255.255.0 if you want to be on both
networks.  The student wireless network is on a 10.?.?.? subnet
which can cause problems with connecting to the robot.

If running Bazel on the `download_stripped` target does not work for
the IP address you're using for the roborio or the raspberry pi, it
probably means that the robot and laptop are on different subnets.
They need to be on the same subnet for the laptop to connect to the
robot.  Connecting can be confirmed by using ping.
```
ping roboRIO-971-frc.local
```
or
```
ping 10.9.71.2
```

If this does not work, perhaps the roboRIO has not been configured to
have a static IP address.  Use a USB cable to connect from a Windows
laptop to the roboRIO and use a web browser (Chrome is preferred,
IE/Edge is not-- see [this technical
note](https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-web-dashboard.html))
to configure the roboRIO to have a static IP address of 10.9.71.2.
Browse to http://roborio-971-frc.local or http://172.22.11.2.  Click
on the "Ethernet" icon on the left, select "Static" for the "Configure
IPv4 Address" option.  Set the "IPv4 Address" to 10.9.71.2. Set the
"Subnet Mask" to "255.0.0.0".  Finally click on "Save" at the bottom
of the screen.  If you have trouble using an Ethernet cable, try using
a USB cable (USB A->B).

Another option is to configure the laptop to have a link-local connection
by using the "Network Settings" GUI.  The laptop will then be on the
same subnet in the address range of 169.254.0.0 to 169.254.255.255.
This might only work over Etherne, not USB, and if the robot
does *not* have a static IP address set and there is no DHCP server
assigning an IP address to the roboRIO.  This implies that the roboRIO
will also have a 169.254.*.* IP addresss, and that the only simple way
to figure it out is to use mDNS.
