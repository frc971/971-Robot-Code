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
      'target_name': 'CameraHTTPStreamer',
      'type': 'executable',
      'sources': [
        'HTTPStreamer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
    {
      'target_name': 'CameraReader',
      'type': 'executable',
      'sources': [
        'Reader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
  ],
}
