# Introduction
This is FRC Team 971's main code repository. There are `README*` files throughout the source tree documenting specifics for their respective folders.

## Contributing
All development of AOS is done on our gerrit instance at https://software.frc971.org/gerrit with github being a read-only mirror.  We are happy to add external contributors.  If you are interested, reach out to Austin Schuh or Stephan Massalt and we will help you get access.  In case of disputes over if a patch should be taken or not, Austin has final say.

Submissions must be made under the terms of the following Developer Certificate of Origin.

By making a contribution to this project, I certify that:

        (a) The contribution was created in whole or in part by me and I
            have the right to submit it under the open source license
            indicated in the file; or

        (b) The contribution is based upon previous work that, to the best
            of my knowledge, is covered under an appropriate open source
            license and I have the right under that license to submit that
            work with modifications, whether created in whole or in part
            by me, under the same open source license (unless I am
            permitted to submit under a different license), as indicated
            in the file; or

        (c) The contribution was provided directly to me by some other
            person who certified (a), (b) or (c) and I have not modified
            it.

        (d) I understand and agree that this project and the contribution
            are public and that a record of the contribution (including all
            personal information I submit with it, including my sign-off) is
            maintained indefinitely and may be redistributed consistent with
            this project or the open source license(s) involved.

To do this, add the following to your commit message.  Gerrit will enforce that all commits have been signed off.

	Signed-off-by: Random J Developer <random@developer.example.org>

Git has support for adding Signed-off-by lines by using `git commit -s`, or you can setup a git commit hook to automatically sign off your commits.  [Stack Overflow](https://stackoverflow.com/questions/15015894/git-add-signed-off-by-line-using-format-signoff-not-working) has instructions for how to do this if you are interested.

## Access to the code
The main central location for our code is our [Gerrit](https://www.gerritcodereview.com/) server at https://software.frc971.org/gerrit. To download a copy of the 971 code on your computer, follow these steps:
  1. Fill out an SVN account request form to get an SVN account.
    1. Mention that you are part of the software team and need Gerrit access
    2. It is recommended to use your firstname in all lowercase for your username
  2. Go to our [Gerrit server](https://software.frc971.org/gerrit) and create an account.
  3. Contact Brian Silverman or Stephan Massalt with your SVN username to get access to the code in Gerrit.
  4. When you log into Gerrit the first time, please [add your Email Address](http://software.frc971.org/gerrit/settings/#EmailAddresses)
  5. Add your SSH key to Gerrit in order to be able to check out code
    1. If you don't already have an ssh key, you can create one using `ssh-keygen`.  This will create a public/private key pair-- the default location for your public key will be `~/.ssh/id_rsa.pub`
    2. Log into Gerrit and go to `Settings->SSH Keys` and paste your public key into the `New SSH Key` text box and clicking on `ADD NEW SSH KEY`
  6. Install `git`: `sudo apt install git`
  7. Go to [the 971-Robot-Code project in Gerrit](https://software.frc971.org/gerrit/#/admin/projects/971-Robot-Code) and run the command to Download the 971-Robot-Code repository.
    1. We recommend downloading the code via SSH using the `clone with commit-msg hook` command
    2. NOTE: Running with the option to `clone with commit-msg hook` will save you trouble later.

To learn more about git, open a terminal and run `man git`, or see [git(1)](https://manpages.debian.org/buster/git-man/git.1.en.html) (especially the NOTES section).


## Prerequisites
The instructions below assume you have the following:
  1. A host computer with an appropriate OS or VM to compile the 971 code using Bazel
    1. The currently supported operating system for building the code is amd64 Debian Buster.
    2. It is likely to work on any `x86\_64 GNU/Linux` system (e.g., Ubuntu 20.04), but that's not at all well-tested.
  2. Your favorite text editor installed, e.g., `vim`, `emacs`
  3. Access to the 971-Robot-Code repository and have downloaded the source code
  4. The ability to `ssh` into target CPU's like the roborio and Raspberry Pi


## Building the code
We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive docs](https://docs.bazel.build/versions/master/build-ref.html), including a nice [build encyclopedia reference](https://docs.bazel.build/versions/master/be/overview.html), and does a great job with fast, correct incremental rebuilds.

There are a couple options for building code that are given here-- setting up either your own computer, or using the frc971 build server.

### Steps to set up a computer to build the code:
  1. Install any Bazel version:
     1. Check to see if the version of Linux you're running has an apt package for Bazel: `apt-cache search bazel` or just try `sudo apt install bazel`
     2. More likely, you'll need to install manually-- see [here](https://docs.bazel.build/versions/master/install-ubuntu.html).  We recommend using `apt-key` instead of `gnupg` in setting up the key:
        1. Step 1: Add Bazel distribution URI as a package source
           ```
           sudo apt install curl
           curl -fsSL https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
           echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
           ```
        2. Step 2: Install Bazel
           ```
           sudo apt update && sudo apt install bazel
           ```

  2. Install the required packages:
     ```sh
     sudo apt-get update
     sudo apt-get install python
     ```
  3. Change settings to allow Bazel's sandboxing to work-- follow the directions in `doc/frc971.conf`.  For example, the commands to do this would be:
     1. `sudo cp doc/frc971.conf /etc/sysctl.d/`
     2. `sudo sysctl --system`

  4. In order to run certain tests, you need to give yourself
     permissions--follow the "Setup real-time niceties" section of
     [aos/events/README.md](aos/events/README.md).

### Setting up access to a workspace on the build server
In order to use the build server, you'll first need to get ssh access set up.  (NOTE: you don't need to do any of the other setup steps done for your own computer, since things like `bazel`, `python`, etc. are already installed on the build server)
  1. Use ssh-keygen to create a public and private key.
```console
# In windows:
# Important use Powershell!
cd ~/.ssh
ssh-keygen -f id_971_rsa
chmod 600 id_971_rsa
```
```console
# In Linux and MacOS:
cd ~
ssh-keygen -f ~/.ssh/id_971_rsa
chmod 600 ./.ssh/id_971_rsa
```
  2. Send the contents of id_971_rsa.pub to Stephan Massalt along with the password that you want to use. WAIT for feedback, as he needs to setup the account.
```console
cat ~/.ssh/id_971_rsa.pub
# Then send the stuff that gets printed to Stephan via slack.
```
  3. Once you hear back from Stephan, test SSH.
```console
ssh [user]@build.frc971.org -p 2222 -i ~/.ssh/id_971_rsa
```
  4. If that doesnt work, then send the error msg to #coding However, if it does then use the `exit` command and then SSH tunnel.
```console
ssh [user]@build.frc971.org -p 2222 -i ~/.ssh/id_971_rsa -L 9971:127.0.0.1:3389
```
  5. So at this point you run the Remote Desktop app in Windows.  Once
you get there, all you need to do is put `127.0.0.1:9971` for the
computer name, and use your SVN username.  Once you get connected,
accept the server certificate and then enter your password that you
gave Stephan. (It's either something unique or your SVN pwd) Then
select the Default panel config.  You can exit the Remote Desktop if
you are good w/ the raw cmd line interface.  And for future logins all
you have to do is tunnel and then login using the app.  Now if you
have a graphical application that you are developing (e.g., spline
UI), then you have to run the build command in the Remote Desktop
application.
  6. Very important: In order for you to be able to commit, you need
  to configure your email address in `git`.  To do this, run the
  following command, replacing `<YOUR EMAIL>` with the email that you are
  using for Gerrit:
```console
git config --global user.email "<YOUR EMAIL>"
```
If there are any questions, post to the #coding Slack channel so that other people who may reach the same issue can refer back to that.


### Bazel commands for building, testing, and deploying the code:
  * Build and test everything (on the host system, for the roborio target-- note, this may take a while):
```
bazel test //...
bazel build --config=roborio -c opt //...
```
  * Build the code for a specific robot:
```console
# For the roborio:
bazel build --config=roborio -c opt //y2020/...
```
```
# For the raspberry pi:
bazel build --config=armv7 -c opt //y2020/...
```

  * Configuring a roborio: Freshly imaged roboRIOs need to be configured to run the 971 code
at startup.  This is done by using the setup_roborio.sh script.
```console
bazel run -c opt //frc971/config:setup_roborio -- roboRIO-XXX-frc.local
```

  * Download code to a robot:
```console
# For the roborio
bazel run --config=roborio -c opt //y2020:download_stripped -- roboRIO-971-frc.local
```
This assumes the roborio is reachable at `roboRIO-971-frc.local`.  If that does not work, you can try with a static IP address like `10.9.71.2` (see troubleshooting below)
```console
# For the raspberry pi's
bazel run --config=armv7 -c opt //y2020:pi_download_stripped -- 10.9.71.101
```
NOTE:
  1. The raspberry pi's require that you have your ssh key installed on them in order to copy code over
  2. They are configured to use the IP addresses 10.X.71.Y, where X is 9, 79, 89, or 99 depending on the robot number (971, 7971, 8971, or 9971, respectively), and Y is 101, 102, etc for pi #1, #2, etc.

  * Downloading specific targets to the robot
    1. Generally if you want to update what's running on the robot, you can use the `download_stripped` (or `pi_download_stripped`) targets.  These will rsync only the changed files, and so are pretty efficient.
    2. If you have a need to compile a specific module, you can build stripped versions of the individual modules by adding "_stripped" to the module name.  For example, to build the calibration code (`//y2020/vision:calibration`) for the pi (`armv7`), run:
    ```console
    bazel run --config=armv7 -c opt //y2020/vision:calibration_stripped
    ```
    You will then need to manually copy the resulting file over to the robot.

## Code reviews
We want all code to at least have a second person look it over before it gets merged into the `master` branch. Gerrit has [extensive documentation on starting reviews](https://software.frc971.org/gerrit/Documentation/user-upload.html).  There is also a good [intro User Guide](https://software.frc971.org/gerrit/Documentation/intro-user.html) and an [intro to working with Gerrit](https://gerrit-review.googlesource.com/Documentation/intro-gerrit-walkthrough.html) and [Gerrit workflows](https://docs.google.com/presentation/d/1C73UgQdzZDw0gzpaEqIC6SPujZJhqamyqO1XOHjH-uk)

TL;DR: Make and commit your changes, do `git push origin HEAD:refs/for/master`, and then click on the provided link to add reviewers.  If you just upload a change without adding any reviewers, it might sit around for a long time before anybody else notices it.

[git-review](http://manpages.debian.org/cgi-bin/man.cgi?query=git-review) can make the upload process simpler.



### Some other useful packages <TODO: Need to review these>
These aren't strictly necessary to build the code, but Michael found the
additional tools provided by these packages useful to install when working with
the code on May 13, 2018.


```console
# Get some useful packages including git and subversion.
   apt-get update
   apt-get install git git-gui vim-gtk3
   apt-get install vim-doc git-doc exim4-doc-html yapf
   apt-get install bazel clang-format
   apt-get install python avahi-daemon
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
# Also may want to install `buildifier` for formatting BUILD files
```

### Creating ssh aliases

It is also handy to alias logins to the raspberry pi's by adding lines like this to your ~/.ssh/config file:
```console
Host pi-7971-2
    User pi
    ForwardAgent yes
    HostName 10.79.71.102
    StrictHostKeyChecking no
```
or, for the roborio:
```
Host roborio-971
    User admin
    HostName 10.9.71.2
    StrictHostKeyChecking no
```
This allows you to use the alias to `ping`, `ssh`, or run commands like:
```
# Download code to robot #7971's raspberry pi #2
bazel run --config=armv7 -c opt //y2020:download_stripped -- pi-7971-2
```

### Roborio Kernel Traces

Currently (as of 2020.02.26), top tends to produce misleading statistics. As
such, you can get more useful information about CPU usage by using kernel
traces. Sample usage:
```console
# Note that you will need to install the trace-cmd package on the roborio.
# This may be not be a trivial task.
# Start the trace
trace-cmd start -e sched_switch -e workqueue
# Stop the trace
trace-cmd stop
# Save the trace to trace.dat
trace-cmd extract
```
You can then scp the `trace.dat` file to your computer and run `kernelshark
trace.dat` (may require installing the `kernelshark` apt package).


### Notes on troubleshooting network setup
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
James thinks this will only work over Ethernet (i.e., not USB; he is not
sure what will happen if you attempt this over USB), and if the robot
does *not* have a static IP address set and there is no DHCP server
assigning an IP address to the roboRIO.  James says to also note that this
implies that the roboRIO will also have a 169.254.*.* IP addresss, and
that the only simple way to figure it out is to use mDNS.


### Other resources
  1. Intro to [AOS](./aos/README.md), our robot Operating System
  2. Introductory example: [ping/pong](./aos/events/README.md)
  3. Example of [logging](./aos/events/README.md)

TODOs:
  1. Add more on networking setup and troubleshooting
  2. Move Roborio Kernel Traces out of here, maybe into documentation/
  3. Currently requires `apt install libsigsegv2`, but this should be temporary
