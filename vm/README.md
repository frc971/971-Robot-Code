Setting up a Virtual Machine (VM)
================================================================================
This document tries to document three ways in which you can set up a
development environment. The third is essentially the same as the second so
this document will just focus on the first and second way.

1. Use the vagrant scripts to automatically create everything.
2. Create a VM manually and set it up manually.
3. Install Debian natively and set it up manually.

Using the vagrant scripts requires more setup, but generally is more hands-off.
Setting up a VM manually can be more rewarding and interesting, but requires
more manual steps and generally takes longer.

Command line knowledge
--------------------------------------------------------------------------------
Some basic knowledge of using your terminal is required. On Windows, cmd.exe is
a good start. I would recommend setting up git-bash because it will resemble
the environment inside the VM much more closely than cmd.exe.

Whenever you see `$` in a code segment below, please type that into your
terminal. `$` is just a generic representation of what we call the "command
prompt". On Windows' cmd.exe it would look something like this:

    C:\Users\YourName>

On a UNIX-like Operating System (e.g. Linux, OSX) it may look more like this:

    yourname@hostname ~ $

The `$` is just a shortcut to represent any style of prompt. When you see
something like `$ echo Hello` below please type everything after the `$ ` into
your terminal. In this case that means typing "echo Hello" and pressing Enter.

Access to the Gerrit repository
--------------------------------------------------------------------------------
In order to use the setup scripts you'll need access to the 971-Robot-Code
repository on gerrit:
<https://robotics.mvla.net/gerrit/#/admin/projects/971-Robot-Code>

Please ask your mentors about getting access. If you already have access,
great!

In general I recommend setting up an SSH key to pull from and push to the
repository. You can also use HTTPS-based passwords, but it's a little more
annoying to use in the long term. See the "How to Generate an SSH key" section
on [gerrit's SSH page](https://robotics.mvla.net/gerrit/#/settings/ssh-keys)
for more details.


Using Vagrant
================================================================================

Requirements
--------------------------------------------------------------------------------
These requirements apply to all Operating Systems. The individual setup steps
may differ. For example, on Debian you can run `apt-get install virtualbox`
whereas on Windows and OSX you have to download a dedicated installer.

1. Basic knowledge of the command line. See the "Command line knowledge"
   section above.

2. Install Vagrant <https://www.vagrantup.com/downloads.html>
    - Please install this from the website as anything in, say, an apt repo
      would be quite outdated.

3. Install VirtualBox <https://www.virtualbox.org/wiki/Downloads>
    - On Debian you should install this via `apt-get` since it's integrated
      really well. On another OS please use the downloaded installer.

4. Install git <https://git-scm.com/downloads>
    - On Debian you should install this via `apt-get`. On another OS please use
      the downloaded installer.
    - On Windows, I would recommend installing something called "git-bash" at
      the same time. It will provide you with a better terminal experience than
      `cmd.exe` to perform the majority of the remaining steps.

5. Add `vagrant`, `VBoxManage`, and `git` to your PATH.
    - This is most likely already done by the installation binaries.
      It's added to the system path.
    - To test this, type these commands in a terminal:

            $ vagrant --version
            Vagrant 1.8.1
            $ VBoxManage --version
            5.0.14r105127
            $ git --version
            git version 2.11.0

    - You may need to log out and back in for the PATH modifications to take
      effect.

Usage
--------------------------------------------------------------------------------
1. Check this folder out on your computer somewhere.

        $ git clone ssh://USERNAME@robotics.mvla.net:29418/971-Robot-Code

   where you replace `USERNAME` with your own username. Keep in mind that you
   need your SVN and Gerrit account set up for this to work. Ask the mentors or
   other students about this if you don't have one yet.

2. Go into the directory and build the VM.

        $ cd 971-Robot-Code/vm/
        $ vagrant up

   Some errors during the `vagrant up` process can be addressed by
   re-provisioning the vagrant box. This is useful if, for example, an
   `apt-get` invocation timed out and caused the provisioning process to abort.

        $ vagrant provision

3. Once built, reboot the VM so it starts the GUI properly.

        $ vagrant reload

4. You can then log in and open a terminal. The username and password are both
   `user`.

5. Download the code.

        $ git clone https://USERNAME@robotics.mvla.net/gerrit/971-Robot-Code
        $ cd 971-Robot-Code

6. Build the code.

        $ bazel build //y2017/...

   Replace `//y2017` with the appropriate year's folder. For 2018 the build
   target would be `//y2018` for example.


Setting up a VM manually
================================================================================
This section is lacking a lot of detail, but that's largely because you can
find most of the information on other websites in a lot more detail.

Requirements
--------------------------------------------------------------------------------
1. Basic knowledge of the command line. See the "Command line knowledge"
   section above.

2. Install VirtualBox <https://www.virtualbox.org/wiki/Downloads>

   See the details from the "Using Vagrant" section above.

3. Download a Debian 8 ISO. You can find one online. The following link may or
   may not work:
   <https://cdimage.debian.org/cdimage/archive/8.9.0/amd64/iso-cd/debian-8.9.0-amd64-netinst.iso>

Usage
--------------------------------------------------------------------------------
1. Start VirtualBox and create a new VM. Make sure to mount the ISO in the
   virtual CD/DVD drive of the VM.

   There are a lot of guides online for creating a VM and can change between
   VirtualBox versions. If VirtualBox asks for the type of VM, select "Debian
   64-bit".

2. Boot the VM and go through the guided installation steps to install Debian.
   Once the installation completes, reboot to boot into your newly installed
   system. This will be part of the guided installation.

3. Check this folder out on your computer somewhere.

        $ git clone ssh://USERNAME@robotics.mvla.net:29418/971-Robot-Code

   where you replace `USERNAME` with your own username. Keep in mind that you
   need your SVN and Gerrit account set up for this to work. Ask the mentors or
   other students about this if you don't have one yet.

4. Run the setup script so you can start building our code.

        $ cd 971-Robot-Code/vm/
        $ sudo ./setup_code_building.sh

5. Now you can build code. For example, to build all the 2017 code.

        $ bazel build //y2017/...
