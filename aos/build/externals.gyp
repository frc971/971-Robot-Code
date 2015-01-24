# This file has targets for various external libraries.
# download_externals.sh makes sure that all of them have been downloaded.
{
  'variables': {
    'externals': '<(AOS)/../output/downloaded',
    'externals_abs': '<!(readlink -f ../../output/downloaded)',
    'compiled': '<(externals)/../compiled-<(ARCHITECTURE)<(EXTERNALS_EXTRA)',
    'compiled_abs': '<(externals_abs)/../compiled-<(ARCHITECTURE)<(EXTERNALS_EXTRA)',

# These versions have to be kept in sync with the ones in download_externals.sh.
    'eigen_version': '3.2.1',
    'gtest_version': '1.6.0-p2',
    'ctemplate_version': '129',
    'gflags_version': '2.0',
    'compiler_rt_version': 'RELEASE_32_final',
    'libevent_version': '2.0.21',
    'libcdd_version': '094g',
    'stm32flash_commit': '8399fbe1baf2b7d097746786458021d92895d71b',

    'allwpilib': '<(AOS)/externals/allwpilib',
    'forwpilib': '<(AOS)/externals/forwpilib',
  },
  'targets': [
    {
      'target_name': 'WPILib',
      'type': 'static_library',
      'variables': {
        'header_dirs': [
          '<(forwpilib)',
          '<(allwpilib)/wpilibc/wpilibC++/include',
          '<(allwpilib)/wpilibc/wpilibC++Devices/include',
          '<(allwpilib)/hal/include',
          '<(allwpilib)/hal/lib/Athena/FRC_FPGA_ChipObject',
          '<(allwpilib)/hal/lib/Athena',
        ],
      },
      'include_dirs': [
        '<@(header_dirs)'
      ],
      'cflags': [
        '-Wno-error=unused-parameter',
        '-Wno-error=switch-enum',
      ],
      'sources': [
        '<!@(ls <(allwpilib)/wpilibc/wpilibC++/src/*.cpp)',
        '<!@(ls <(allwpilib)/wpilibc/wpilibC++Devices/src/*.cpp)',
        '<!@(ls <(allwpilib)/wpilibc/wpilibC++Devices/src/Internal/*.cpp)',
        '<!@(ls <(allwpilib)/hal/lib/Athena/*.cpp)',
        '<!@(ls <(allwpilib)/hal/lib/Athena/ctre/*.cpp)',
        '<(forwpilib)/dma.cc',
      ],
      'link_settings': {
        'library_dirs': [
          '<(allwpilib)/ni-libraries',
        ],
        'libraries': [
          '-lpthread',
          '-lFRC_NetworkCommunication',
          '-lRoboRIO_FRC_ChipObject',
          '-lNiFpgaLv',
          '-lNiFpga',
          '-lNiRioSrv',
          '-lspi',
          '-li2c',
        ],
      },
      'direct_dependent_settings': {
        'include_dirs': [
          '<@(header_dirs)'
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
        '<(externals)/gtest-<(gtest_version)/src/gtest-all.cc',
            '<(externals)/gtest-<(gtest_version)/fused-src/gtest/gtest_main.cc',
      ],
      'include_dirs': [
        '<(externals)/gtest-<(gtest_version)',
      ],
      'dependencies': [
        'gtest_prod',
      ],
      'export_dependent_settings': [
        'gtest_prod',
      ],
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
  ],
}
