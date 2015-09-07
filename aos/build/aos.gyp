# Converted to bazel
# This file has all of the aos targets.
{
  'targets': [
    # A target for things used by the logging implementation (except die) to
    # depend on that allows linking successfully with logging calls. However,
    # executables containing targets that depend on this still need a dependency
    # on logging somewhere or else they won't link.
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
          'dependencies': [
            '<(AOS)/linux_code/linux_code.gyp:complex_thread_local',
          ],
        }],
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/common/libc/libc.gyp:aos_strerror',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/libc/libc.gyp:aos_strerror',
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
            '<(AOS)/common/common.gyp:time',
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
      'export_dependent_settings': [
        'logging_interface',
      ],
    },
  ],
}
