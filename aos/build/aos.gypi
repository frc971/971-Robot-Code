# This file gets passed to gyp with -I so that it gets included everywhere.
{
  'variables': {
    'AOS': '<(DEPTH)/aos',
# A directory with everything in it ignored from source control.
    'TMPDIR': '<(DEPTH)/aos/build/temp',
    'aos_abs': '<!(readlink -f <(DEPTH)/aos)', # for use in non-path contexts
# the .gyp file that has targets for the various external libraries
    'EXTERNALS': '<(AOS)/build/externals.gyp',
# the directory that gets rsynced to the atom
    'rsync_dir': '<(PRODUCT_DIR)/outputs',
# The directory that loadable_module and shared_library targets get put into
# There's a target_conditions that puts loadable_modules here and
#   shared_librarys automatically get put here.
    'so_dir': '<(PRODUCT_DIR)/lib',
# the directory that executables that depend on <(EXTERNALS):gtest get put into
    'test_dir': '<(PRODUCT_DIR)/tests',
# 'executable' for the atom and 'static_library' for the cRIO
# Useful for targets that should either be an executable or get compiled into
# a .out file depending on the current platform.
#   'aos_target': platform-dependent,
  },
  'conditions': [
    ['OS=="crio"', {
        'make_global_settings': [
          ['CC', '<!(readlink -f <(AOS)/build/crio_cc)'],
          ['CXX', '<!(readlink -f <(AOS)/build/crio_cxx)'],
        ],
        'variables': {
          'aos_target': 'static_library',
        },
      }, {
        'make_global_settings': [
          ['CC', '<!(which arm-linux-gnueabihf-gcc-4.7)'],
          ['CXX', '<!(which arm-linux-gnueabihf-g++-4.7)'],
        ],
        'variables': {
          'aos_target': 'executable',
        },
      }
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
      # Set this to 1 if this file isn't a test that should get run by
      # `build.sh tests`.
      'is_special_test%': 0,
    },
    'conditions': [
      ['DEBUG=="yes"', {
          'cflags': [
            '-O0',
          ],
        }, {
          'cflags': [
            '-O3',
          ],
          'ldflags': [
            '-O3',
          ],
          'conditions': [['OS=="crio"', {
              'cflags': [
                '-fstrength-reduce',
                '-fno-builtin',
                '-fno-strict-aliasing',
              ],
            }, {
              'cflags': [
                '-mcpu=cortex-a8',
                '-mfpu=neon',

                '-fstack-protector-all',
              ],
            }
          ]],
        }
      ],
      ['OS=="crio"', {
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
            ['_type=="loadable_module" or _type=="shared_library"', {
                'ldflags': [
# Support loading other shared objects that are in the same directory but not
#   the shared object load path. Required for using the swig-generated libs.
                  '-Wl,-rpath=\\$$ORIGIN',
                ],
              }
            ],
          ],
          'ldflags': [
            '-pthread',
          ],
          'cflags': [
            '-pthread',

            '-Wunused-local-typedefs',

            # Give macro stack traces when they blow up.
            # TODO(brians): Re-enable this once they fix the bug where it
            # sometimes doesn't show you the top-most (aka most useful)
            # line of code.
            #'-ftrack-macro-expansion',
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
        }
      ]
    ],
  },
}
