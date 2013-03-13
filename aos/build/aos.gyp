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
            '<(AOS)/crio/Talon.cpp',
          ],
        }, {
          'libaos_source_files': [
            '<(AOS)/atom_code/async_action/AsyncAction_real.cpp',
          ],
        }
      ],
    ],
  },
  'targets': [
    {
      'target_name': 'logging',
      'type': 'static_library',
      'sources': [
        '<(AOS)/common/logging/logging_impl.cc',
      ],
      'conditions': [
        ['OS=="crio"', {
          'sources': [
            '<(AOS)/crio/logging/crio_logging.cc',
          ],
          'dependencies': [
            '<(EXTERNALS):WPILib',
            # TODO(brians): socket should only depend on the interface
            #'<(AOS)/common/network/network.gyp:socket',
          ]
        }, {
          'sources': [
            '<(AOS)/atom_code/logging/atom_logging.cc',
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
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:die',
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
      'target_name': 'aos_core',
      'type': 'static_library',
      'sources': [
        #'<(AOS)/aos_core.h'
      ],
      'conditions': [
        ['OS=="atom"', {
          'dependencies': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
            '<(AOS)/atom_code/atom_code.gyp:init',
          ],
          'export_dependent_settings': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
            '<(AOS)/atom_code/atom_code.gyp:init',
          ],
        }, {
          'dependencies': [
            '<(EXTERNALS):WPILib',
          ],
          'export_dependent_settings': [
            '<(EXTERNALS):WPILib',
          ],
        }]
      ],
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
