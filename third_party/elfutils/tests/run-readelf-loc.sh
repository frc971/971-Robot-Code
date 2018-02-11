#! /bin/sh
# Copyright (C) 2013 Red Hat, Inc.
# This file is part of elfutils.
#
# This file is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# elfutils is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

. $srcdir/test-subr.sh

# - hello.c
# int say (const char *prefix);
#
# static char *
# subject (char *word, int count)
# {
#   return count > 0 ? word : (word + count);
# }
#
# int
# main (int argc, char **argv)
# {
#    return say (subject (argv[0], argc));
# }
#
# - world.c
# static int
# sad (char c)
# {
#   return c > 0 ? c : c + 1;
# }
#
# static int
# happy (const char *w)
# {
#   return sad (w[1]);
# }
#
# int
# say (const char *prefix)
# {
#   const char *world = "World";
#   return prefix ? sad (prefix[0]) : happy (world);
# }
#
# gcc -g -O2 -c hello.c
# gcc -g -O2 -c world.c
# gcc -g -o testfileloc hello.o world.o

testfiles testfileloc

# Process values as offsets from base addresses and resolve to symbols.
testrun_compare ${abs_top_builddir}/src/readelf --debug-dump=loc --debug-dump=ranges \
  testfileloc<<\EOF

DWARF section [33] '.debug_loc' at offset 0xd2a:

 CU [     b] base: 0x0000000000400480 <main>
 [     0] range 0, d
          0x0000000000400480 <main>..
          0x000000000040048c <main+0xc>
           [ 0] reg5
 [    23] range 5, d
          0x0000000000400485 <main+0x5>..
          0x000000000040048c <main+0xc>
           [ 0] reg5

 CU [    e0] base: 0x00000000004004a0 <say>
 [    46] range 12, 1a
          0x00000000004004b2 <say+0x12>..
          0x00000000004004b9 <say+0x19>
           [ 0] breg5 0

DWARF section [34] '.debug_ranges' at offset 0xd94:

 CU [     b] base: 0x0000000000400480 <main>
 [     0] range 0, 2
          0x0000000000400480 <main>..
          0x0000000000400481 <main+0x1>
          range 5, d
          0x0000000000400485 <main+0x5>..
          0x000000000040048c <main+0xc>

 CU [    e0] base: 0x00000000004004a0 <say>
 [    30] range d, f
          0x00000000004004ad <say+0xd>..
          0x00000000004004ae <say+0xe>
          range 12, 1a
          0x00000000004004b2 <say+0x12>..
          0x00000000004004b9 <say+0x19>
EOF

# Don't resolve addresses to symbols.
testrun_compare ${abs_top_builddir}/src/readelf -N --debug-dump=loc --debug-dump=ranges \
  testfileloc<<\EOF

DWARF section [33] '.debug_loc' at offset 0xd2a:

 CU [     b] base: 0x0000000000400480
 [     0] range 0, d
          0x0000000000400480..
          0x000000000040048c
           [ 0] reg5
 [    23] range 5, d
          0x0000000000400485..
          0x000000000040048c
           [ 0] reg5

 CU [    e0] base: 0x00000000004004a0
 [    46] range 12, 1a
          0x00000000004004b2..
          0x00000000004004b9
           [ 0] breg5 0

DWARF section [34] '.debug_ranges' at offset 0xd94:

 CU [     b] base: 0x0000000000400480
 [     0] range 0, 2
          0x0000000000400480..
          0x0000000000400481
          range 5, d
          0x0000000000400485..
          0x000000000040048c

 CU [    e0] base: 0x00000000004004a0
 [    30] range d, f
          0x00000000004004ad..
          0x00000000004004ae
          range 12, 1a
          0x00000000004004b2..
          0x00000000004004b9
EOF

# Produce "raw" unprocessed content.
testrun_compare ${abs_top_builddir}/src/readelf -U --debug-dump=loc --debug-dump=ranges \
  testfileloc<<\EOF

DWARF section [33] '.debug_loc' at offset 0xd2a:

 CU [     b] base: 0x0000000000400480
 [     0] range 0, d
           [ 0] reg5
 [    23] range 5, d
           [ 0] reg5

 CU [    e0] base: 0x00000000004004a0
 [    46] range 12, 1a
           [ 0] breg5 0

DWARF section [34] '.debug_ranges' at offset 0xd94:

 CU [     b] base: 0x0000000000400480
 [     0] range 0, 2
          range 5, d

 CU [    e0] base: 0x00000000004004a0
 [    30] range d, f
          range 12, 1a
EOF

exit 0
