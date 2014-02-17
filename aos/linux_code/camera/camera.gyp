{
  'targets': [
    {
      'target_name': 'buffers',
      'type': 'static_library',
      'sources': [
        'Buffers.cpp',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:scoped_message_ptr',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:scoped_message_ptr',
      ],
    },
    {
      'target_name': 'CameraHTTPStreamer',
      'type': 'executable',
      'sources': [
        'HTTPStreamer.cpp',
      ],
      'dependencies': [
        'buffers',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'CameraReader',
      'type': 'executable',
      'sources': [
        'Reader.cpp',
      ],
      'dependencies': [
        'buffers',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
      ],
    },
  ],
}
