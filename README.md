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

## Table of contents
* [Pathway for new members](#pathway-for-new-members)
    * [Learning C++](#1-learning-c)
        * [XRP](#11-xrp)
    * [Gaining code access](#2-gaining-code-access)
    * [The codelab](#3-the-codelab)
* [Code Reviews](#code-reviews)
    * [Creating SSH Aliases](#creating-ssh-aliases)
* [Codelab](#codelab)
* [Other Information](#other-information)
    * [Roborio Kernel Traces](#roborio-kernel-traces)
    * [Notes on troubleshooting network setup](#notes-on-troubleshooting-network-setup)
    * [LSP Setup for Rust](#lsp-setup-for-rust)
    * [Other resources](#other-resources)

## Pathway for new members

### 1. Learning C++

C++ is what we use to program a majority of the code which runs on the robot.
We generally recommend [this course by codeacademy](https://www.codecademy.com/learn/learn-c-plus-plus) which goes over the basics of C++ as well as OOP.

#### 1.1 XRP

If you are unsure what to work on yet you can start on the [XRP Platform](https://docs.wpilib.org/en/stable/docs/xrp-robot/index.html).
Which basically lets you use smaller robots to learn about robotics through WPILib.

### 2. Gaining code access

You can follow [these steps](./documentation/tutorials/getting-started.md) to access the code.

There is also a document which goes over the basics of using both gerrit and git [here](./documentation/tutorials/git-and-gerrit-basics.md)

### 3. The codelab

Once you have an understanding of C++ and have gone through using all the steps needed to access the code you can start on the codelab, which is a directory living in `frc971/codelab/`. Which goes over a lot of the important introductory information that you need to know to get started when programming the robot.
You can read the [README](./frc971/codelab/README.md), which gives you introduces you to the codelab as well as the concepts you need to understand first.

## Code reviews
We want all code to at least have a second person look it over before it gets merged into the `main` branch. Gerrit has [extensive documentation on starting reviews](https://gerrit-review.googlesource.com/Documentation/user-upload.html).  There is also a good [intro User Guide](https://gerrit-review.googlesource.com/Documentation/intro-user.html) and an [intro to working with Gerrit](https://gerrit-review.googlesource.com/Documentation/intro-gerrit-walkthrough.html) and [Gerrit workflows](https://docs.google.com/presentation/d/1C73UgQdzZDw0gzpaEqIC6SPujZJhqamyqO1XOHjH-uk)

TL;DR: Make and commit your changes, do `git push origin HEAD:refs/for/main`, and then click on the provided link to add reviewers.  If you just upload a change without adding any reviewers, it might sit around for a long time before anybody else notices it.

[git-review](http://manpages.debian.org/cgi-bin/man.cgi?query=git-review) can make the upload process simpler.

To download a commit you can press d or click download on gerrit, then select fetch and paste the command in your terminal. Once you've done that you should switch to a new branch in order to make working on multiple commits easier.
You can do this by doing `git switch -c branch_name`, the branch name can be anything, and will only be public for you, so organize in whatever way you find easy to model.

When programming in C++ we follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html)

Furthermore, you can run `bazel run //tools/lint:run-ci` to fix linter errors which appear on buildkite.

### Some other useful packages
These aren't strictly necessary to build the code, but Michael found the
additional tools provided by these packages useful to install when working with
the code on May 13, 2018.

> **Note**: These are for local install, most of these tools will be available on the build server

```bash
apt install git
apt install yapf3
apt install python3
apt install bazel
apt install clang-format
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

## Other Information

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

### LSP Setup for Rust

You can run `bazel run //tools:gen_rust_project` to generate a rust-project.json
file which rust-analyzer will pick up. You will need to execute this rule
periodically as it will be outdated whenever the BUILD files change.

> **Note** that there's currently no way to tell rust-analyzer *how* to compile
the code, so while it will give you completion support, go to definition, and
other niceties, it won't show compilation errors or warnings at this point.


### Other resources
  1. Intro to [AOS](./aos/README.md), our robot Operating System
  2. Introductory example: [ping/pong](./aos/events/README.md)
  3. Example of [logging](./aos/events/README.md)
