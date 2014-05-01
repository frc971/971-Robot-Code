# This file gets passed to gyp with -I so that it gets included everywhere.
{
  'variables': {
    'AOS': '<(DEPTH)/aos',
# A directory with everything in it ignored from source control.
    'TMPDIR': '<(DEPTH)/aos/build/temp',
    'aos_abs': '<!(readlink -f <(DEPTH)/aos)', # for use in non-path contexts
# the .gyp file that has targets for the various external libraries
    'EXTERNALS': '<(AOS)/build/externals.gyp',
# the directory that gets rsynced to the target
    'rsync_dir': '<(PRODUCT_DIR)/outputs',
# The directory that loadable_module and shared_library targets get put into
# There's a target_conditions that puts loadable_modules here and
#   shared_librarys automatically get put here.
    'so_dir': '<(PRODUCT_DIR)/lib',
# the directory that executables that depend on <(EXTERNALS):gtest get put into
    'test_dir': '<(PRODUCT_DIR)/tests',
  },
  'conditions': [
    ['PLATFORM=="crio"', {
        'make_global_settings': [
          ['CC', '<!(readlink -f <(AOS)/build/crio_cc)'],
          ['CXX', '<!(readlink -f <(AOS)/build/crio_cxx)'],
        ],
      }
    ], ['PLATFORM=="linux-arm-gcc"', {
        'make_global_settings': [
          ['CC', '<!(which arm-linux-gnueabihf-gcc-4.7)'],
          ['CXX', '<!(which arm-linux-gnueabihf-g++-4.7)'],
        ],
      },
    ], ['PLATFORM=="linux-arm-clang"', {
        'variables': {
          'arm-clang-symlinks': '<!(realpath -s <(AOS)/build/arm-clang-symlinks)',
          'arm-clang-sysroot': '<(arm-clang-symlinks)/sysroot',
          'platflags': [
            '-target', 'armv7a-linux-gnueabihf',
            '-mfloat-abi=hard',
            '--sysroot=<(arm-clang-sysroot)',

            #-mhwdiv=arm,thumb
          ],
        },
        'make_global_settings': [
          ['CC', '<(arm-clang-symlinks)/bin/clang'],
          ['CXX', '<(arm-clang-symlinks)/bin/clang++'],
        ],
        'target_defaults': {
          'cflags': [
            '<@(platflags)',
          ],
          'cflags_cc': [
            '-isystem', '<(arm-clang-sysroot)/usr/include/c++/4.7.2',
            '-isystem', '<(arm-clang-sysroot)/usr/include/c++/4.7.2/arm-linux-gnueabihf',
          ],
          'ldflags': [
            '<@(platflags)',
          ],
        },
      },
    ], ['PLATFORM=="linux-amd64-clang"', {
        'make_global_settings': [
          ['CC', '<!(which clang)'],
          ['CXX', '<!(which clang++)'],
        ],
      },
    ], ['PLATFORM=="linux-amd64-gcc"', {
      },
    ],
  ],
  'target_defaults': {
    'defines': [
      '__STDC_FORMAT_MACROS',
      '_FORTIFY_SOURCE=2',
      '__STDC_CONSTANT_MACROS',
      '__STDC_LIMIT_MACROS',
    ],
    'ldflags': [
      '-pipe',
    ],
    'cflags': [
      '-pipe',

      '-Wall',
      '-Wextra',
      '-Wswitch-enum',
      '-Wpointer-arith',
      '-Wstrict-aliasing=2',
      '-Wcast-qual',
      '-Wcast-align',
      '-Wwrite-strings',
      '-Wtype-limits',
      '-Wsign-compare',
      '-Wformat=2',
      '-Werror',

      '-ggdb3',
    ],
    'cflags_c': [
      '-std=gnu99',
    ],
    'include_dirs': [
      '<(DEPTH)',
    ],
    # These have to be here because apparently gyp evaluates target_conditions
    # even if the target is never used.
    'variables': {
      # Set this to 1 to disable rsyncing the file to the target.
      'no_rsync%': 0,
      # Set this to 1 if this file is a test that should not be run by
      # `build.py tests`.
      'is_special_test%': 0,
    },
    'conditions': [
      ['DEBUG=="yes"', {
          'cflags': [
            '-O0',
          ],
        }, {
          'cflags': [
            # TODO(brians): -O4 for clang to enable LTO?
            '-O3',
            '-fomit-frame-pointer',
          ],
          'ldflags': [
            '-O3',
          ],
          'conditions': [['PLATFORM=="crio"', {
              'cflags': [
                '-fstrength-reduce',
                '-fno-builtin',
                '-fno-strict-aliasing',
              ],
            }],
            ['ARCHITECTURE=="arm"', {
              'cflags': [
                '-mcpu=cortex-a8',
                '-mfpu=neon',
              ],
            }],
            ['ARCHITECTURE=="amd64"', {
              'cflags': [
                '-fstack-protector-all',
              ],
            }],
          ]
        }
      ],
      ['PLATFORM=="linux-arm-gcc" and DEBUG=="yes"', {
          'cflags': [
              # GCC doesn't like letting us use r7 (which is also the frame
              # pointer) to pass the syscall number to the kernel even when
              # it's marked as clobbered.
              # See <https://bugzilla.mozilla.org/show_bug.cgi?id=633436> for
              # some more discussion.
            '-fomit-frame-pointer',
          ],
        }
      ],
      ['PLATFORM=="crio"', {
          'target_conditions': [
            ['_type=="shared_library"', {
                'ldflags': [
                  '-r',
                  '-nostdlib',
                  '-Wl,-X',
                ],
              }
            ],
          ],
          'ldflags': [
            '-mcpu=603e',
            '-mstrict-align',
            '-mlongcall',
          ],
          'cflags': [
            # The Freescale MPC5200B (cRIO-FRC) and MPC5125 (cRIO-FRC II) both
            # have MPC603e cores according to Freescale docs.
            '-mcpu=603e',
            '-mstrict-align',
            '-mlongcall',
            '-isystem', '<(aos_abs)/../output/downloaded/gccdist/WindRiver/gnu/3.4.4-vxworks-6.3/x86-win32/lib/gcc/powerpc-wrs-vxworks/3.4.4/include/',
            '-isystem', '<(aos_abs)/../output/downloaded/gccdist/WindRiver/vxworks-6.3/target/h/',
            '-isystem', '<(aos_abs)/../output/downloaded/gccdist/WindRiver/gnu/3.4.4-vxworks-6.3/x86-win32/include/c++/3.4.4/',
            '-isystem', '<(aos_abs)/../output/downloaded/gccdist/WindRiver/gnu/3.4.4-vxworks-6.3/x86-win32/include/c++/3.4.4/powerpc-wrs-vxworks/',
            '-isystem', '<(WIND_BASE)/target/h',
            '-isystem', '<(WIND_BASE)/target/h/wrn/coreip',
          ],
          'cflags_cc': [
            '-std=gnu++0x',
          ],
          'defines': [
            'CPU=PPC603',
            'TOOL_FAMILY=gnu',
            'TOOL=gnu',
            '_WRS_KERNEL',
            '__PPC__',
# This tells eigen to not do anything with alignment at all. See
# <http://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html> for
# details. It really doesn't like to work without this.
            'EIGEN_DONT_ALIGN',
# prevent the vxworks system headers from being dumb and #defining min and max
            'NOMINMAX',
          ],
        }, {
          'target_conditions': [
# default to putting outputs into rsync_dir
            ['no_rsync==0 and _type!="static_library"', {
                'product_dir': '<(rsync_dir)',
              },
            ],
            ['_type=="loadable_module"', {
                'product_dir': '<(so_dir)',
              }
            ],
          ],
          'ldflags': [
            '-pthread',
          ],
          'cflags': [
            '-pthread',
            '-fno-exceptions',
          ],
          'cflags_cc': [
            '-std=gnu++11',
          ],
          'defines': [
            '_FILE_OFFSET_BITS=64',
          ],
          'libraries': [
            '-lm',
            '-lrt',
          ],
          'conditions': [
            ['COMPILER=="gcc"', {
                'cflags': [
                  '-Wunused-local-typedefs',
                ],
              },
            ], ['COMPILER=="clang"', {
                'cflags': [
                  '-fcolor-diagnostics',
                ],
                'defines': [
                  # To work around <http://llvm.org/bugs/show_bug.cgi?id=13530>.
                  '__float128=void',
                  # This tells clang's optimizer the same thing.
                  '__builtin_assume_aligned(p, a)=(((uintptr_t(p) % (a)) == 0) ? (p) : (__builtin_unreachable(), (p)))',
                ],
              },
            ],
          ],
        }
      ]
    ],
  },
}
