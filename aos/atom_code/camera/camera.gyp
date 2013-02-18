{
  'targets': [
    {
      'target_name': 'aos_camera',
      'type': 'loadable_module',
      'sources': [
        'jni.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:aos_shared_lib',
        '<(AOS)/common/network/network.gyp:socket_so',
        '<(AOS)/common/common.gyp:timing_so',
        '<(AOS)/atom_code/messages/messages.gyp:messages_so',
        'private_aos_camera_jar',
        '<(EXTERNALS):libjpeg',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:aos_shared_lib',
        '<(AOS)/common/network/network.gyp:socket_so',
        '<(AOS)/common/common.gyp:timing_so',
        '<(AOS)/atom_code/messages/messages.gyp:messages_so',
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
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
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
        '<(AOS)/atom_code/atom_code.gyp:init',
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
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
  ],
}
