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
The currently supported operating system for building the code is amd64 Debian Stretch. It is likely to work on any x86\_64 GNU/Linux system, but that's not at all well-tested.

We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive](https://docs.bazel.build/versions/master/build-ref.html) [docs](https://docs.bazel.build/versions/master/be/overview.html) and does a nice job with fast, correct increment rebuilds.

### Steps to set up a computer to build the code:
  0. Set up the required APT repositories:
     Download
	 [frc971.list](http://robotics.mvla.net/files/frc971/packages/frc971.list)
	 and put it in `/etc/apt/sources.list.d/`.
  1. Install the required packages:
```console
apt-get update
apt-get install bazel python ruby
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
bazel build --cpu=roborio --compilation_mode=opt //y2018/...
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
bazel run --cpu=roborio --compilation_mode=opt //y2018:download -- admin@roboRIO-971-frc.local
# If this does not work, try
bazel run --cpu=roborio --compilation_mode=opt //y2018:download -- admin@10.9.71.2
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
setup_roborio.sh roboRIO-971-frc.local
```

# ============================ Notes for running under Stretch ===================
#
# Parker and Austin helped me compile the code under the Stretch version of Debian.
# Below are my notes on what was done differently.  I started with a cleanly
# installed Debian Stretch OS.  Michael Schuh, May 13, 2018.
#

# Install different packages than those listed above:
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

# Make the following changes to the configuration files.
# The one change to the graph.cc file is to update it to work with current
# complilers.  The other changes are required to work with a newer version
# of clang.
#
# The sed commands can be run by copying and pasting them to the command line.
# The sed commands will change the file contents and replace the file with the
# updated changes.

#
# aos/build/queues/compiler.rb
#
sed -i -e 's/clang-format-3.5/clang-format-3.8/' aos/build/queues/compiler.rb

diff --git a/aos/build/queues/compiler.rb b/aos/build/queues/compiler.rb
index 51646702..6a273ae9 100644
--- a/aos/build/queues/compiler.rb
+++ b/aos/build/queues/compiler.rb
@@ -77,7 +77,7 @@ def parse_args(globals,args)
 end
 def format_pipeline(output)
   read_in, write_in = IO.pipe()
-  child = Process.spawn('/usr/bin/clang-format-3.5 --style=google',
+  child = Process.spawn('/usr/bin/clang-format-3.8 --style=google',
                         {:in=>read_in, write_in=>:close,
                          :out=>output.fileno})
   read_in.close

#
# tools/cpp/CROSSTOOL
#
sed -e 's?clang/3.6.2?clang/3.8.1?' \
    -e 's/clang-3.6/clang-3.8/' \
    -e 's/clang_3.6/clang_3.8/' \
    -e 's/llvm-3.6/llvm-3.8/' \
    -e 's?clang/3.6?clang/3.8?' \
    -i tools/cpp/CROSSTOOL

diff --git a/tools/cpp/CROSSTOOL b/tools/cpp/CROSSTOOL
index d4e7fbb1..caa2f633 100644
--- a/tools/cpp/CROSSTOOL
+++ b/tools/cpp/CROSSTOOL
@@ -83,7 +83,7 @@ toolchain {
   tool_path { name: "compat-ld" path: "/usr/bin/ld" }
   tool_path { name: "cpp" path: "/usr/bin/cpp" }
   tool_path { name: "dwp" path: "/usr/bin/dwp" }
-  tool_path { name: "gcc" path: "/usr/bin/clang-3.6" }
+  tool_path { name: "gcc" path: "/usr/bin/clang-3.8" }
   tool_path { name: "gcov" path: "/usr/bin/gcov" }
   # C(++) compiles invoke the compiler (as that is the one knowing where
   # to find libraries), but we provide LD so other rules can invoke the linker.
@@ -104,10 +104,10 @@ toolchain {
   cxx_builtin_include_directory: '/usr/include/x86_64-linux-gnu/c++/4.9'
   cxx_builtin_include_directory: '/usr/include/c++/4.9/backward'
   cxx_builtin_include_directory: '/usr/local/include'
-  cxx_builtin_include_directory: '/usr/lib/llvm-3.6/lib/clang/3.6.2/include'
+  cxx_builtin_include_directory: '/usr/lib/llvm-3.8/lib/clang/3.8.1/include'
   cxx_builtin_include_directory: '/usr/include/x86_64-linux-gnu'
   cxx_builtin_include_directory: '/usr/include'
-  cxx_builtin_include_directory: '/usr/lib/clang/3.6.2/include'
+  cxx_builtin_include_directory: '/usr/lib/clang/3.8.1/include'

   linker_flag: "-lstdc++"
   linker_flag: "-B/usr/bin/"
@@ -535,7 +535,7 @@ toolchain {
 }

 toolchain {
-  abi_version: "clang_3.6"
+  abi_version: "clang_3.8"
   abi_libc_version: "glibc_2.19"
   builtin_sysroot: ""
   compiler: "clang"
@@ -578,7 +578,7 @@ toolchain {

   compiler_flag: "-nostdinc"
   compiler_flag: "-isystem"
-  compiler_flag: "/usr/lib/clang/3.6/include"
+  compiler_flag: "/usr/lib/clang/3.8/include"
   compiler_flag: "-isystem"
   compiler_flag: "external/linaro_linux_gcc_4_9_repo/lib/gcc/arm-linux-gnueabihf/4.9.3/include"
   compiler_flag: "-isystem"
@@ -604,7 +604,7 @@ toolchain {
   cxx_builtin_include_directory: "%package(@linaro_linux_gcc_4_9_repo//lib/gcc/arm-linux-gnueabihf/4.9.3/include)%"
   cxx_builtin_include_directory: "%package(@linaro_linux_gcc_4_9_repo//lib/gcc/arm-linux-gnueabihf/4.9.3/include-fixed)%"
   cxx_builtin_include_directory: "%package(@linaro_linux_gcc_4_9_repo//arm-linux-gnueabihf/include)%/c++/4.9.3"
-  cxx_builtin_include_directory: '/usr/lib/clang/3.6/include'
+  cxx_builtin_include_directory: '/usr/lib/clang/3.8/include'

   linker_flag: "-target"
   linker_flag: "armv7a-arm-linux-gnueabif"

#
# To compile the code for the robot, libisl.so.10 is needed.  It is not
# available on stretch package servers so download the amd64 jessie package
# from https://packages.debian.org/jessie/libisl10
# and install it with dpkg.
# This installs
#   /usr/lib/x86_64-linux-gnu/libisl.so.10
#   /usr/lib/x86_64-linux-gnu/libisl.so.10.2.2
#   /usr/share/doc/libisl10/changelog.Debian.gz
#   /usr/share/doc/libisl10/changelog.gz
#   /usr/share/doc/libisl10/copyright
#
wget http://ftp.us.debian.org/debian/pool/main/i/isl/libisl10_0.12.2-2_amd64.deb
dpkg -i libisl10_0.12.2-2_amd64.deb

# After doing this, this compile command should work.
time bazel build --cpu=roborio --compilation_mode=opt //y2018:download
# On Michael's Lenovo ThinkPad X270 with an
#    Intel(R) Core(TM) i7-6500U CPU @ 2.50GHz
# compiling the 2018 robot code takes
#   INFO: Elapsed time: 499.177s, Critical Path: 63.33s
#   INFO: Build completed successfully, 4901 total actions
#
#   real	8m19.234s
#   user	0m0.277s
#   sys	0m0.518s


