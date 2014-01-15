{
  'targets': [
    {
      'target_name': 'core',
      'type': 'executable',
      'sources': [
        'core.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'BinaryLogReader',
      'type': 'executable',
      'sources': [
        'BinaryLogReader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
      ],
    },
    {
      'target_name': 'LogStreamer',
      'type': 'executable',
      'sources': [
        'LogStreamer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
      ],
    },
    {
      'target_name': 'LogDisplayer',
      'type': 'executable',
      'sources': [
        'LogDisplayer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
  ],
}
