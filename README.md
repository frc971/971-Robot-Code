# Introduction
This is FRC Team 971's main code repository. There are `README*` files throughout the source tree documenting specifics for their respective folders.

## Access to the code
The main central location for our code is our [Gerrit](https://www.gerritcodereview.com/) server at https://robotics.mvla.net/gerrit. To get a copy of the code on your computer to work with, follow these steps:
  1. Contact Michael Schuh to get an SVN account.
  2. Go to Gerrit and create an account.
  3. Contact Brian Silverman with your SVN username to get access to the code in Gerrit.
  4. Go to [the 971-Robot-Code project in Gerrit](https://robotics.mvla.net/gerrit/#/admin/projects/971-Robot-Code) and run the command.
     Running the `clone with commit-msg hook` command will save you trouble later.

To learn more about git, see git(1) (`man git` or [git(1)](http://manpages.debian.net/cgi-bin/man.cgi?query=git>) (especially the NOTES section).

## Code reviews
We want all code to at least have a second person look over it before it gets merged into the `master` branch. Gerrit has [extensive documentation on starting reviews](https://robotics.mvla.net/gerrit/Documentation/user-upload.html). TL;DR: `git push origin HEAD:refs/for/master` and then click on the link to add reviewers.
If you just upload a change without adding any reviewers, it might sit around for a long time before anybody else notices it.
[git-review](http://manpages.debian.org/cgi-bin/man.cgi?query=git-review) can make the upload process simpler.

## Building the code
The currently supported operating system for building the code is amd64 Debian Buster. It is likely to work on any x86\_64 GNU/Linux system, but that's not at all well-tested.

We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive](https://docs.bazel.build/versions/master/build-ref.html) [docs](https://docs.bazel.build/versions/master/be/overview.html) and does a nice job with fast, correct increment rebuilds.

### Steps to set up a computer to build the code:
  0. Install any Bazel version.  See [here](https://docs.bazel.build/versions/master/install-ubuntu.html)
  1. Install the required packages:
```console
apt-get update
apt-get install bazel python
```
  2. Allow Bazel's sandboxing to work:
     Follow the direction in `doc/frc971.conf`.

### Some useful Bazel commands:
  * Build and test everything (on the host system):
```console
bazel test //...
bazel build --cpu=roborio //...
```
  * Build the code for a specific robot:
```console
bazel build --cpu=roborio -c opt //y2019/...
```
  * Download code to a robot:
```console
# First add a host entry in your ~/.ssh/known_hosts file for the roboRIO.
# Do this by ssh'ing into the machine.  If you problems doing this, see
# the notes below for more information on how to connect to the roboRIO.
ssh admin@roboRIO-971-frc.local
# If you see an error like:
#    subprocess.CalledProcessError: Command '['rsync' ...
#    ERROR: Non-zero return code '1' from command: Process exited with status 1
# The above "ssh admin@roboRIO-971-frc.local" step has not been sucessfully completed.
# If the roboRIO has been configued to use a static IP address like 10.9.71.2,
# set the laptop to have an IP address on the 10.9.71.x subnet with a netmask
# of 255.0.0.0.  The ".x" is different than the .2 for the roboRIO or any other
# device on the network.  The driver station uses .5 or .6 so avoid those.
# The radio uses .1 or .50 so avoid those too.  If you are at the school,
# disconnect from the student wireless network or try setting your netmask to
# 255.255.255.0 if you want to be on both networks.  The student wireless
# network is on a 10.?.?.? subnet which can cause problems with connecting to
# the robot.
bazel run --cpu=roborio --compilation_mode=opt //y2018:download_stripped -- admin@roboRIO-971-frc.local
# If this does not work, try
bazel run --cpu=roborio --compilation_mode=opt //y2018:download_stripped -- admin@10.9.71.2
# If this does not work, it probably means that the robot and laptop are on
# different subnets.  They need to be on the same subnet for the laptop to
# connect to the robot.  Connecting can be confirmed by using ping.
ping roboRIO-971-frc.local
# or
ping 10.9.71.2
# If this does not work, perhaps the roboRIO has not been configured to have
# a static IP address.  Use a USB cable to connect from a Windows laptop to
# the roboRIO and use Internet Explorer (IE) to configure the roboRIO
# to have a static IP address of 10.9.71.2.  Inside IE, browse to
# http://roborio-971-frc.local or http://172.22.11.2.  Click on the "Ethernet"
# icon on the left, select "Static" for the "Configure IPv4 Address" option.
# Set the "IPv4 Address" to 10.9.71.2. Set the "Subnet Mask" to "255.0.0.0".
# Finally click on "Save" at the bottom of the screen.  If you have trouble
# using an Ethernet cable, try using a USB cable.  USB cables are much
# more reliable for connecting than using a Ethernet cabe.  USB cables work
# for connecting to the robot on Windows and Linux computers.
# Another option is to configure the laptop to have a link-local connection
# by going using the "Network Settings" GUI.  The laptop will then be on the
# same subnet in the address range of 169.254.0.0 to 169.254.255.255.
# James thinks this will only work over Ethernet (i.e., not USB; he is not
# sure what will happen if you attempt this over USB), and if the robot
# does *not* have a static IP address set and there is no DHCP server
# assigning an IP address to the roboRIO.  James says to also note that this
# implies that the roboRIO will also have a 169.254.*.* IP addresss, and
# that the only simple way to figure it out is to use mDNS.
#
```
  * To configure a freshly imaged roboRIO:
```console
# Freshly imaged roboRIOs need to be configured to run the 971 code
# at startup.  This is done by using the setup_roborio.sh script.
bazel run -c opt //frc971/config:setup_roborio -- roboRIO-XXX-frc.local
```

### Some other useful packages
These aren't strictly necessary to build the code, but Michael found the
additional tools provided by these packages useful to install when working with
the code on May 13, 2018.

```console
# Get some useful packages including git and subversion.
   apt-get update
   apt-get install git subversion ruby python vim-gtk3 subversion-tools
   apt-get install vim-doc git-doc git-gui git-svn exim4-doc-html ruby
   apt-get install python python-scipy python-matplotlib libpython-dev
   apt-get install bazel clang-format-3.8 clang-3.8  openjdk-8-jdk
   apt-get install gfortran libblas-dev liblapack-dev avahi-daemon
# fix a key problem with llvm.
   wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
   apt-get update
# Install apt-file so that packages can be searched
   apt-get install apt-file
   apt-file update
# Install sysstat so that you can tell how many resources are being used during
#   the compile.
   apt-get install sysstat
# iostat is used to observe how hard the disk is being worked and other
#   performance metrics.
   iostat -dx 1
# gitg is a graphical user interface for git.  I find it useful for
# understanding the revision history of the repository and viewing
# log messages and changes.
   apt-get install gitg
```
