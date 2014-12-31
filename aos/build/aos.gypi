# This file gets passed to gyp with -I so that it gets included everywhere.
{
  'variables': {
    'AOS': '<(DEPTH)/aos',
# A directory with everything in it ignored from source control.
    'TMPDIR': '<(DEPTH)/aos/build/temp',
    'aos_abs': '<!(readlink -f <(DEPTH)/aos)', # for use in non-path contexts
# The .gyp file that has targets for the various external libraries.
    'EXTERNALS': '<(AOS)/build/externals.gyp',
# The directory that gets rsynced to the target.
    'rsync_dir': '<(PRODUCT_DIR)/outputs',
# The directory that executables that depend on <(EXTERNALS):gtest get put into.
    'test_dir': '<(PRODUCT_DIR)/tests',

# Stuck into a variable (with a space on the end) to make disabling it easy.
    'ccache': '<!(which ccache) ',

    'disable_sanitizers': [
      # Bad alignment is just slow on x86 and traps on ARM, so we'll find
      # it other ways, and some x86 code does it on purpose.
      'alignment',
    ],
  },
  'conditions': [
    ['PLATFORM=="crio"', {
        'make_global_settings': [
          ['CC', '<!(readlink -f <(AOS)/build/crio_cc)'],
          ['CXX', '<!(readlink -f <(AOS)/build/crio_cxx)'],
        ],
      }
    ], ['PLATFORM=="linux-arm-gcc_frc"', {
        'make_global_settings': [
          ['CC', '<(ccache)<!(which arm-frc-linux-gnueabi-gcc-4.9)'],
          ['CXX', '<(ccache)<!(which arm-frc-linux-gnueabi-g++-4.9)'],
        ],
      },
    ], ['PLATFORM=="linux-arm-gcc"', {
        'make_global_settings': [
          ['CC', '<(ccache)<!(which arm-linux-gnueabihf-gcc-4.7)'],
          ['CXX', '<(ccache)<!(which arm-linux-gnueabihf-g++-4.7)'],
        ],
      },
    ], ['PLATFORM=="linux-arm-clang"', {
        'variables': {
          'arm-clang-symlinks': '<!(realpath -s <(AOS)/build/arm-clang-symlinks)',
          'arm-clang-sysroot': '<(arm-clang-symlinks)/sysroot',
# Flags that should be passed to all compile/link/etc commands.
          'platflags': [
            '-target', 'armv7a-linux-gnueabihf',
            '-mfloat-abi=hard',
            '--sysroot=<(arm-clang-sysroot)',

            # TODO(brians): See if it will run with this enabled.
            #-mhwdiv=arm,thumb
          ],
        },
        'make_global_settings': [
          ['CC', '<(ccache)<(arm-clang-symlinks)/bin/clang'],
          ['CXX', '<(ccache)<(arm-clang-symlinks)/bin/clang++'],
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
          ['CC', '<(ccache)/opt/clang-3.5/bin/clang'],
          ['CXX', '<(ccache)/opt/clang-3.5/bin/clang++'],
        ],
      },
    ], ['PLATFORM=="linux-amd64-gcc"', {
        'make_global_settings': [
          ['CC', '<(ccache)<!(which gcc-4.7)'],
          ['CXX', '<(ccache)<!(which g++-4.7)'],
        ],
      },
    ], ['PLATFORM=="linux-amd64-gcc_4.8"', {
        'make_global_settings': [
          ['CC', '<(ccache)/opt/clang-3.5/bin/gcc'],
          ['CXX', '<(ccache)/opt/clang-3.5/bin/g++'],
        ],
      },
    ], ['SANITIZER!="none"', {
        'target_defaults': {
          'cflags': [
            '-fsanitize=<(SANITIZER)',
          ],
          'ldflags': [
            '-fsanitize=<(SANITIZER)',
          ],
          'defines': [
# GCC doesn't have __has_feature, so we have to use this instead.
            'AOS_SANITIZER_<(SANITIZER)',
          ],
        },
      },
    ], ['SANITIZER!="none" and COMPILER!="gcc"', {
        'target_defaults': {
          'cflags': [
            '-fno-sanitize-recover',
            '-fno-sanitize=<!(echo <(disable_sanitizers) | sed "s/ /,/g")',
          ],
        },
      },
    ], ['SANITIZER!="thread"', {
        'libraries': [
          '<!(readlink -f <(AOS)/../output/compiled-<(ARCHITECTURE)<(EXTERNALS_EXTRA)/gperftools-2.3-prefix/lib/libtcmalloc.a)',
          '<!(readlink -f <(AOS)/../output/compiled-<(ARCHITECTURE)<(EXTERNALS_EXTRA)/libunwind-1.1-prefix/lib/libunwind.a)',
        ],
        'defines': [
          'TCMALLOC',
        ],
      },
    ], ['EXTERNALS_EXTRA=="-fPIE"', {
        'target_defaults': {
          'cflags': [
            '-fPIE',
          ],
          'ldflags': [
            '-fPIE',
          ],
          'link_settings': {
            'ldflags': [
              '-pie',
            ],
          },
        },
      },
    ], ['SANITIZER=="memory"', {
        'target_defaults': {
          'cflags': [
            '-fsanitize-memory-track-origins',
          ],
          'ldflags': [
            '-fsanitize-memory-track-origins',
          ],
        },
      },
    ],
  ],
  'target_defaults': {
    'defines': [
      '__STDC_FORMAT_MACROS',
      '__STDC_CONSTANT_MACROS',
      '__STDC_LIMIT_MACROS',
      'AOS_COMPILER_<!(echo <(FULL_COMPILER) | sed \'s/\./_/g\')',
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
          'defines': [
            'AOS_DEBUG=1',
          ],
          'conditions': [['SANITIZER=="none"', {
              'cflags': [
                '-O0',
              ],
            }, {
              'cflags': [
                '-O1',
              ],
            }
          ]],
        }, { # 'DEBUG=="no"'
          'defines': [
            'AOS_DEBUG=0',
            '_FORTIFY_SOURCE=2',
          ],
          'cflags': [
            '-O3',
            '-fomit-frame-pointer',
          ],
          'ldflags': [
            '-O3',
          ],
          'conditions': [['PLATFORM=="crio"', {
# Copied from stuff that I think started with the supplied Makefiles.
              'cflags': [
                '-fstrength-reduce',
                '-fno-builtin',
                '-fno-strict-aliasing',
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
      ['OS=="linux" and ARCHITECTURE=="arm" and COMPILER=="gcc" and DEBUG=="yes"', {
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
      ['ARCHITECTURE=="arm" and FULL_COMPILER!="gcc_frc"', {
        'cflags': [
          '-mcpu=cortex-a8',
          '-mfpu=neon',
        ],
        'ldflags': [
          '-mcpu=cortex-a8',
          '-mfpu=neon',
        ],
      }],
      ['ARCHITECTURE=="arm" and FULL_COMPILER=="gcc_frc"', {
        'cflags': [
          '-mcpu=cortex-a9',
          '-mfpu=neon',
          '-mfloat-abi=softfp',
        ],
        'ldflags': [
          '-mcpu=cortex-a9',
          '-mfpu=neon',
          '-mfloat-abi=softfp',
        ],
      }],
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
# Prevent the vxworks system headers from being dumb and #defining min and max.
            'NOMINMAX',
          ],
        }, { # 'PLATFORM!="crio"'
          'target_conditions': [
# Default to putting outputs into rsync_dir.
            ['no_rsync==0 and _type!="static_library"', {
                'product_dir': '<(rsync_dir)',
              },
            ],
          ],
          'ldflags': [
            '-pthread',
          ],
          'cflags': [
            '-pthread',
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
                'defines': [
                  '__has_feature(n)=0'
                ],
              },
            ], ['COMPILER=="clang"', {
                'cflags': [
                  '-fcolor-diagnostics',
                  '-fmessage-length=80',
                  '-fmacro-backtrace-limit=0',
                ],
                'defines': [
                  # This tells clang's optimizer the same thing.
                  '__builtin_assume_aligned(p, a)=({ const typeof(p) my_p_ = (p); ((((uintptr_t)my_p_ % (a)) == 0u) ? my_p_ : (__builtin_unreachable(), (my_p_))); })',
                ],
              },
            ],
          ],
        }
      ]
    ],
  },
}
