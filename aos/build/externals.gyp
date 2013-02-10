# This file has targets for various external libraries.
# download_externals.sh makes sure that all of them have been downloaded.
{
  'variables': {
    'externals': '<(AOS)/externals',
    'externals_abs': '<!(readlink -f ../externals)',

# These versions have to be kept in sync with the ones in download_externals.sh.
    'eigen_version': '3.0.5',
    'gtest_version': '1.6.0-p1',
    'onejar_version': '0.97',
    'ctemplate_version': '2.2',
  },
  'targets': [
    {
# does nothing when OS!="crio"
      'target_name': 'WPILib',
      'type': 'none',
      'conditions': [['OS=="crio"', {
            'direct_dependent_settings': {
              'cflags': [
                '-isystem', '<(aos_abs)/externals/WPILib',
              ],
              'link_settings': {
                'libraries': [
                  '<(aos_abs)/externals/WPILib/WPILib.a',
                ],
              },
            },
        }]],
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
# TODO(brians) convert this to downloading + building
      'target_name': 'libevent',
      'type': 'none',
      'link_settings': {
        'libraries': ['-levent'],
      },
    },
    {
      'target_name': 'eigen',
      'type': 'none',
      'direct_dependent_settings': {
        'include_dirs': ['<(externals)/eigen-<(eigen_version)'],
      },
    },
    {
      'target_name': 'libjpeg',
      'type': 'none',
      'direct_dependent_settings': {
        'libraries': ['<(externals_abs)/libjpeg/lib/libjpeg.a'],
      },
    },
    {
      'target_name': 'gtest',
      'type': 'static_library',
      'sources': [
        '<(externals)/gtest-<(gtest_version)/fused-src/gtest/gtest-all.cc',
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
      'include_dirs': [
        '<(externals)/gtest-<(gtest_version)',
        '<(externals)/gtest-<(gtest_version)/include'
      ],
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
        'libraries': ['<(externals)/ctemplate-<(ctemplate_version)-prefix/lib/libctemplate.a'],
      },
      'direct_dependent_settings': {
        'include_dirs': ['<(externals)/ctemplate-<(ctemplate_version)-prefix/include'],
      },
    },
  ],
}
