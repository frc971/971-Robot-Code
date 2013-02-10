# This file has all of the aos targets.
# For the cRIO, shared_library means to build a .out file, NOT a shared library.
#   This means that depending on shared libraries doesn't work very well.
# Shared libraries don't seem to be supported by the powerpc-wrs-vxworks
# tools and gyp doesn't like a static_library that depends on static_librarys.
{
  'variables': {
    'conditions': [
      ['OS=="crio"', {
          'libaos_source_files': [
            '<!@(find <(AOS)/crio/controls <(AOS)/crio/messages <(AOS)/crio/motor_server <(AOS)/crio/shared_libs -name *.c -or -name *.cpp -or -name *.cc)',
            '<(AOS)/crio/Talon.cpp',
            '<(AOS)/common/die.cc',
          ],
        }, {
          'libaos_source_files': [
            '<(AOS)/atom_code/camera/Buffers.cpp',
            '<(AOS)/atom_code/async_action/AsyncAction_real.cpp',
            '<(AOS)/atom_code/init.cc',
            '<(AOS)/atom_code/ipc_lib/mutex.cpp',
            '<(AOS)/common/die.cc',
          ],
        }
      ],
    ],
  },
  'targets': [
    {
      'target_name': 'logging',
      'type': 'static_library',
      'conditions': [
        ['OS=="crio"', {
          'sources': [
            '<(AOS)/crio/logging/crio_logging.cpp',
          ],
          'dependencies': [
            '<(EXTERNALS):WPILib',
          ]
        }, {
          'sources': [
            '<(AOS)/atom_code/logging/atom_logging.cpp'
          ],
          'dependencies': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
          'export_dependent_settings': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ]
        }],
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
# Private to make Brian happy.  Don't use elsewhere in so targets or risk things
# breaking.
      'target_name': 'aos_swig',
      'type': 'static_library',
      'sources': [
        '<(AOS)/aos.swig',
      ],
      'variables': {
        'package': 'aos',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../build/swig.gypi'],
    },
    {
      'target_name': 'libaos',
      'type': 'static_library',
      'sources': ['<@(libaos_source_files)'],
      'sources/': [['exclude', '_test\.c[cp]*$']],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        'logging',
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(EXTERNALS):WPILib',
      ],
      'conditions': [
        ['OS=="atom"', {
          'dependencies': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
        }]
      ],
    },
    {
      'target_name': 'aos_shared_lib',
      'type': 'shared_library',
      'sources': ['<@(libaos_source_files)'],
      'sources/': [['exclude', '_test\.c[cp]*$']],
      'variables': {'no_rsync': 1},
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:queues_so',
        '<(AOS)/common/common.gyp:queues',
        'aos_swig',
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:queues_so',
        '<(EXTERNALS):WPILib',
        'aos_swig',
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': [
            'aos_shared_lib',
          ],
        },
      },
    },
    {
# A target that has all the same dependencies as libaos and aos_shared_lib
#   without any queues so that the queues can get the necessary headers without
#   creating circular dependencies.
      'target_name': 'aos_internal_nolib',
      'type': 'none',
      'dependencies': [
        'aos/ResourceList.h',
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        'aos/ResourceList.h',
        '<(EXTERNALS):WPILib',
      ],
    },
    {
      'target_name': 'aos/ResourceList.h',
      'type': 'static_library',
      'direct_dependent_settings': {
        'include_dirs': [
          '<(SHARED_INTERMEDIATE_DIR)/ResourceList',
        ],
      },
      'hard_dependency': 1,
      'actions': [
        {
          'variables': {
            'script': '<(AOS)/build/gen_resource_list.rb'
          },
          'action_name': 'gen_aos_ResourceList_h',
          'inputs': ['<(script)'],
          'outputs': ['<(SHARED_INTERMEDIATE_DIR)/ResourceList/aos/ResourceList.h'],
          'message': 'Generating',
          'action': ['ruby', '<(script)', '<(SHARED_INTERMEDIATE_DIR)/ResourceList/aos',],
        },
      ],
    },
  ],
}
