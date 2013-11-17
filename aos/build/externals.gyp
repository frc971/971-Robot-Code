# This file has targets for various external libraries.
# download_externals.sh makes sure that all of them have been downloaded.
{
  'variables': {
    'externals': '<(AOS)/../output/downloaded',
    'externals_abs': '<!(readlink -f ../../output/downloaded)',
    'compiled': '<(externals)/../compiled-i386',
    'compiled_abs': '<(externals_abs)/../compiled-i386',

# These versions have to be kept in sync with the ones in download_externals.sh.
    'eigen_version': '3.1.3',
    'gtest_version': '1.6.0',
    'onejar_version': '0.97',
    'ctemplate_version': '129',
    'gflags_version': '2.0',
    'libusb_version': '1.0.9',
    'libusb_apiversion': '1.0',
    'compiler_rt_version': 'RELEASE_32_final',
    'libevent_version': '2.0.21',
    'libcdd_version': '094g',
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
      'direct_dependent_settings': {
        'include_dirs': [
          '<(AOS)/externals/WPILib',
        ],
      },
    },
    {
      'target_name': 'onejar',
      'type': 'none',
      'direct_dependent_settings': {
        'variables': {
          'onejar_jar': '<(externals_abs)/one-jar-boot-<(onejar_version).jar',
        },
      },
    },
    {
      'target_name': 'javacv',
      'type': 'none',
      'variables': {
        'javacv_dir': '<(externals_abs)/javacv-bin',
      },
      'direct_dependent_settings': {
        'include_dirs': [
          '/usr/lib/jvm/default-java/include',
          '/usr/lib/jvm/default-java/include/linux',
        ],
        'variables': {
          'classpath': [
            '<(javacv_dir)/javacv.jar',
            '<(javacv_dir)/javacpp.jar',
            '<(javacv_dir)/javacv-linux-x86.jar',
          ],
        },
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
      'conditions': [['OS=="crio"', {
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
          ['_type=="executable"', {
              'product_dir': '<(test_dir)',
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
      'target_name': 'libusb',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/libusb-<(libusb_version)-prefix/lib/libusb-<(libusb_apiversion).a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled)/libusb-<(libusb_version)-prefix/include'],
      },
    },
    {
      'target_name': 'libcdd',
      'type': 'none',
      'link_settings': {
        'libraries': ['<(compiled_abs)/libcdd-<(libcdd_version)-prefix/lib/libcdd.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(compiled_abs)/libcdd-<(libcdd_version)-prefix/include'],
      },
    },
  ],
  'includes': [
    'libgcc-additions/libgcc-additions.gypi',
  ],
}
