{
  'targets': [
    {
      'target_name': 'aos_camera',
      'type': 'loadable_module',
      'sources': [
        'jni.cpp',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket_so',
        '<(AOS)/common/common.gyp:timing_so',
        'private_aos_camera_jar',
        '<(EXTERNALS):libjpeg',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket_so',
        '<(AOS)/common/common.gyp:timing_so',
        'private_aos_camera_jar',
      ],
    },
    {
      'target_name': 'private_aos_camera_jar',
      'dependencies': [
        '<(EXTERNALS):javacv',
      ],
      'variables': {
        'srcdirs': ['java'],
        'gen_headers': ['aos.Natives'],
      },
      'export_dependent_settings': [
        '<(EXTERNALS):javacv',
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': ['aos_camera'],
        },
      },
      'includes': ['../../build/java.gypi'],
      'hard_dependency': 1,
    },
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
