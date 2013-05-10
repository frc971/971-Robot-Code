{
  'targets': [
    {
      'target_name': 'messages_so',
      'type': 'shared_library',
      'variables': {'no_rsync': 1},
      'sources': [
        'DriverStationDisplay.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': [
            'messages_so',
          ],
        },
      },
    },
    {
      'target_name': 'messages',
      'type': 'static_library',
      'sources': [
        'DriverStationDisplay.cpp',
      ],
      'dependencies': [
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
      ],
      'export_dependent_settings': [
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
      ],
    },
  ],
}
