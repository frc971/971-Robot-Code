{
  'targets': [
    {
      'target_name': 'socket_so',
      'type': 'shared_library',
      'variables': {'no_rsync': 1},
      'sources': [
        'ReceiveSocket.cpp',
        'SendSocket.cpp',
        'Socket.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
      ],
      'conditions': [
        ['OS=="crio"', {
          'dependencies': [
            '<(EXTERNALS):WPILib',
          ]}
        ],
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': [
            'socket_so',
          ],
        },
      },
    },
    {
      'target_name': 'socket',
      'type': 'static_library',
      'sources': [
        'ReceiveSocket.cpp',
        'SendSocket.cpp',
        'Socket.cpp',
      ],
      'conditions': [
        ['OS=="crio"', {
          'dependencies': [
            '<(EXTERNALS):WPILib',
          ]}
        ],
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
      ],
    },
  ],
}
