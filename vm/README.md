Requirements
--------------------------------------------------------------------------------
1. Install Vagrant <https://www.vagrantup.com/downloads.html>

1. Install VirtualBox <https://www.virtualbox.org/wiki/Downloads>

1. Add `vagrant` and `VBoxManage` to your PATH.
    - This is most likely already done by the installation binaries.
      It's added to the system path.
    - To test this, type these commands in a terminal:

            ~$ vagrant --version
            Vagrant 1.8.1
            ~$ VBoxManage --version
            5.0.14r105127

    - You may need to log out and back in for the path modifications to take
      effect.

1. On my Jessie installation I had to apply the following patch before I could
   successfully shut down and reboot the VM.

        --- /opt/vagrant/embedded/gems/gems/vagrant-1.7.4/plugins/guests/debian8/cap/halt.rb    2015-07-17 13:15:13.000000000 -0700
        +++ new_halt.rb 2015-11-18 20:11:29.003055639 -0800
        @@ -4,7 +4,7 @@
               class Halt
                 def self.halt(machine)
                   begin
        -            machine.communicate.sudo("shutdown -h -H")
        +            machine.communicate.sudo("systemctl poweroff")
                   rescue IOError
                     # Do nothing, because it probably means the machine shut down
                     # and SSH connection was lost.

Usage
--------------------------------------------------------------------------------
1. Check this folder out on your computer somewhere.

        svn co https://robotics.mvla.net/svn/frc971/2016/trunk/src/vagrant_dev_vm

1. Go into the directory and build the VM.

        vagrant up

1. Some errors during the `vagrant up` process can be addressed by
   re-provisioning the vagrant box. This is useful if, for example, an
   `apt-get` invocation timed out and caused the provisioning process to abort.

        vagrant provision

1. Once build, reboot the VM so it starts the GUI properly.

        vagrant reload

1. You can then log in and open a terminal. The username and password are both
   `user`.

1. Download the code and build it.

        git clone https://USERNAME@robotics.mvla.net/gerrit/971-Robot-Code
        cd 971-Robot-Code
        bazel build //y2016/... -- $(cat NO_BUILD_AMD64)

   where USERNAME is the same username you use to log into SVN.
