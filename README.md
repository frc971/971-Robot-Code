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
The currently supported operating system for building the code is amd64 Debian Jessie. It is likely to work on any x86\_64 GNU/Linux system, but that's not at all well-tested.

We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive](http://bazel.io/docs/build-ref.html) [docs](http://bazel.io/docs/build-encyclopedia.html) and does a nice job with fast, correct increment rebuilds.

Steps to set up a computer to build the code:
  0. Set up the required APT repositories:
     Download
	 [frc971.list](http://robotics.mvla.net/files/frc971/packages/frc971.list)
	 and
	 [llvm.org.list](http://robotics.mvla.net/files/frc971/packages/llvm.org.list)
	 and put them in `/etc/apt/sources.list.d/`.
  1. Install the required packages:
```console
apt-get install python libpython-dev bazel ruby clang-format-3.5 clang-3.6 gfortran libblas-dev liblapack-dev python-scipy python-matplotlib
```
  2. Allow Bazel's sandboxing to work

Some useful Bazel commands:
  * Build and test everything (on the host system):
```console
bazel test //... -- $(cat NO_BUILD_AMD64)
bazel build --cpu=roborio //... -- $(cat NO_BUILD_ROBORIO)
```
    The NO_BUILD_{AMD64,ROBORIO} files contain lists of the targets which are intentionally not built for the various CPUs.
  * Build the code for a specific robot:
```console
bazel build --cpu=roborio --compilation_mode=opt //y2015/...
```
  * Download code to a robot:
```console
bazel run --cpu=roborio --compilation_mode=opt //y2015:download roboRIO-971.local
```
