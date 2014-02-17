# This file has all of the aos targets.
# For the cRIO, shared_library means to build a .out file, NOT a shared library.
#   This means that depending on shared libraries doesn't work very well.
# Shared libraries don't seem to be supported by the powerpc-wrs-vxworks
# tools and gyp doesn't like a static_library that depends only on
# other static_librarys.
{
  'targets': [
    # A target for things used by the logging implementation (except die) to
    # depend on that allows linking successfully with logging calls  but has no
    # way to get initialized and so is basically useless unless something else
    # links in the rest of the logging stuff.
    {
      'target_name': 'logging_interface',
      'type': 'static_library',
      'sources': [
        '<(AOS)/common/logging/logging_interface.cc',
      ],
      'conditions': [
        ['OS=="linux"', {
          'sources': [
            '<(AOS)/linux_code/logging/linux_interface.cc',
          ],
        }],
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:die',
      ],
    },
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
          ],
        }],
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:once',
        'logging_interface',
        '<(AOS)/common/common.gyp:queue_types',
      ],
    },
  ],
}
