# This is a control file for creating the clang-3.5 package.
### Commented entries have reasonable defaults.
### Uncomment to edit them.
Source: llvm-toolchain-snapshot
Section: misc
Priority: optional
Homepage: http://www.llvm.org/
Standards-Version: 3.9.2

Package: clang-3.5
Version: 1:3.5~svn201561
Maintainer: FRC Team 971 <spartanrobotics.org>
# Pre-Depends: <comma-separated list of packages>
Depends: libbsd0, libc6, libedit2, libncurses5, libpython2.7, libtinfo5, zlib1g
# Recommends: <comma-separated list of packages>
# Suggests: <comma-separated list of packages>
# Provides: <comma-separated list of packages>
# Replaces: <comma-separated list of packages>
Architecture: amd64
# Copyright: <copyright file; defaults to GPL2>
# Changelog: <changelog file; defaults to a generic changelog>
# Readme: <README.Debian file; defaults to a generic one>
# Extra-Files: <comma-separated list of additional files for the doc directory>
# Files: /opt/clang-3.5/ /opt/
#  <more pairs, if there's more than one file to include. Notice the starting space>
Description: Clang 3.5 with included GCC 4.8 so it actually works.
 This is just 1 massive package with a newer clang that doesn't conflict with
 other stuff like the official clang-3.5 package does.
