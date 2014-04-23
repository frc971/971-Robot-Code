# This file has targets for various external libraries.
# download_externals.sh makes sure that all of them have been downloaded.
{
  'variables': {
    # TODO(brians): Would we not have to do this hackery if we named it externals_path etc?
    'externals': '<(AOS)/../output/downloaded',
    'externals_abs': '<!(readlink -f ../../output/downloaded)',
    'compiled': '<(externals)/../compiled-<(ARCHITECTURE)',
    'compiled_abs': '<(externals_abs)/../compiled-<(ARCHITECTURE)',

# These versions have to be kept in sync with the ones in download_externals.sh.
    'eigen_version': '3.2.1',
    'gtest_version': '1.6.0-p1',
    'ctemplate_version': '129',
    'gflags_version': '2.0',
    'compiler_rt_version': 'RELEASE_32_final',
    'libevent_version': '2.0.21',
    'libcdd_version': '094g',
    'stm32flash_commit': '8399fbe1baf2b7d097746786458021d92895d71b',
  },
  'targets': [
    {
      'target_name': 'WPILib',
      'type': 'static_library',
      'sources': [
        '<!@(find <(AOS)/externals/WPILib/WPILib/ -name *.cpp)',
      ],
      'cflags!': [
        '-Werror',
        '-ggdb3',
        '-O0'
      ],
      'cflags': [
        '-ggdb1',
        '-O3'
      ],
      'include_dirs': [
        '<(AOS)/externals/WPILib',
        '<(AOS)/externals/WPILib/WPILib',
      ],
      'direct_dependent_settings': {
        'cflags': [
          '-isystem', '<(AOS)/externals/WPILib',
          '-isystem', '<(AOS)/externals/WPILib/WPILib',
        ],
      },
    },
    {
      'target_name': 'WPILib-NetworkRobotValues',
      'type': 'static_library',
      'sources': [
        '<(AOS)/externals/WPILib/WPILib/NetworkRobot/NetworkRobotValues.cpp'
      ],
      'include_dirs': [
        '<(AOS)/externals/WPILib',
      ],
      'defines': [
        # Clang doesn't like having register in the hton* macros.
        'register=',
      ],
      'direct_dependent_settings': {
        'include_dirs': [
          '<(AOS)/externals/WPILib',
        ],
        'defines': [
          'register=',
        ],
      },
    },
    {
      'target_name': 'opencv',
      'type': 'none',
      'link_settings': {
        'libraries': [
          '-lopencv_core',
          '-lopencv_imgproc',
        ],
      },
    },
    {
      'target_name': 'libevent',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/libevent-<(libevent_version)-prefix/lib/libevent.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled)/libevent-<(libevent_version)-prefix/include'],
      },
    },
    {
      'target_name': 'eigen',
      'type': 'none',
      'direct_dependent_settings': {
        'cflags': [
          '-isystem', '<(externals)/eigen-<(eigen_version)'
        ],
      },
    },
    {
      'target_name': 'libjpeg',
      'type': 'none',
      'direct_dependent_settings': {
        'libraries': ['<(compiled_abs)/libjpeg/lib/libjpeg.a'],
        'cflags': [
          '-isystem', '<(compiled)',
        ],
      },
    },
    {
# Dependents should only use the "gtest/gtest_prod.h" header.
# This target is NOT the correct one for "aos/common/gtest_prod.h". That one is
#   aos/common/common.gyp:gtest_prod. This target just deals with setting up to
#   use the gtest header.
      'target_name': 'gtest_prod',
      'type': 'static_library',
      'direct_dependent_settings': {
        'include_dirs': [
          '<(externals)/gtest-<(gtest_version)/include'
        ],
      },
    },
    {
      'target_name': 'gtest',
      'type': 'static_library',
      'sources': [
        '<(externals)/gtest-<(gtest_version)/fused-src/gtest/gtest-all.cc',
      ],
      'dependencies': [
        'gtest_prod',
      ],
      'export_dependent_settings': [
        'gtest_prod',
      ],
      'conditions': [['PLATFORM=="crio"', {
            'defines': [
              'GTEST_HAS_TR1_TUPLE=0',
              'GTEST_HAS_STREAM_REDIRECTION=0',
              'GTEST_HAS_POSIX_RE=0', # it only has a broken header...
            ],
            'direct_dependent_settings': {
              'defines': [
                'GTEST_HAS_TR1_TUPLE=0',
                'GTEST_HAS_STREAM_REDIRECTION=0',
                'GTEST_HAS_POSIX_RE=0',
              ],
            },
        }, {
          'sources': [
            '<(externals)/gtest-<(gtest_version)/fused-src/gtest/gtest_main.cc',
          ],
        }]],
      'cflags!': ['-Werror'],
      'direct_dependent_settings': {
        'include_dirs': ['<(externals)/gtest-<(gtest_version)/include'],
        'target_conditions': [
          ['_type=="executable" and is_special_test==0', {
              'product_dir': '<(test_dir)',
            },
          ], ['_type=="executable" and is_special_test==1', {
              'product_dir': '<(test_dir)-special',
            },
          ],
        ],
      },
    },
    {
      'target_name': 'ctemplate',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/ctemplate-<(ctemplate_version)-prefix/lib/libctemplate.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled)/ctemplate-<(ctemplate_version)-prefix/include'],
      },
    },
    {
      'target_name': 'gflags',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/gflags-<(gflags_version)-prefix/lib/libgflags.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled)/gflags-<(gflags_version)-prefix/include'],
      },
    },
    {
      'target_name': 'libcdd',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/libcdd-<(libcdd_version)-prefix/lib/libcdd.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled_abs)/'],
      },
    },
    {
      'target_name': 'stm32flash',
      'type': 'static_library',
      'sources': [
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/init.c',
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/parsers/hex.c',
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/serial_common.c',
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/serial_platform.c',
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/utils.c',
        '<(externals)/stm32flash-<(stm32flash_commit)/stm32flash/stm32.c',
      ],
      'cflags': [
        '-Wno-error',
      ],
      'direct_dependent_settings': {
        'include_dirs': ['<(externals_abs)/stm32flash-<(stm32flash_commit)'],
      },
    },
  ],
  'includes': [
    'libgcc-additions/libgcc-additions.gypi',
  ],
}
