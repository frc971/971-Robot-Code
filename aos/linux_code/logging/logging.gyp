{
  'targets': [
    # linux_* is dealt with by aos/build/aos.gyp:logging.
    {
      'target_name': 'binary_log_writer',
      'type': 'executable',
      'sources': [
        'binary_log_writer.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
        'binary_log_file',
        '<(AOS)/common/common.gyp:queue_types',
      ],
    },
    {
      'target_name': 'log_streamer',
      'type': 'executable',
      'sources': [
        'log_streamer.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
      ],
    },
    {
      'target_name': 'log_displayer',
      'type': 'executable',
      'sources': [
        'log_displayer.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        'binary_log_file',
        '<(AOS)/common/common.gyp:queue_types',
      ],
    },
    {
      'target_name': 'binary_log_file',
      'type': 'static_library',
      'sources': [
        'binary_log_file.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
