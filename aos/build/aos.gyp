# This file has all of the aos targets.
# For the cRIO, shared_library means to build a .out file, NOT a shared library.
#   This means that depending on shared libraries doesn't work very well.
# Shared libraries don't seem to be supported by the powerpc-wrs-vxworks
# tools and gyp doesn't like a static_library that depends on static_librarys.
{
  'targets': [
    {
      'target_name': 'logging',
      'type': 'static_library',
      'sources': [
        '<(AOS)/common/logging/logging_impl.cc',
      ],
      'conditions': [
        ['OS=="linux"', {
          'sources': [
            '<(AOS)/linux_code/logging/linux_logging.cc',
          ],
          'dependencies': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
          ],
          'export_dependent_settings': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
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
  ],
}
