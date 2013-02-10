{
  'targets': [
    {
      'target_name': 'frc971_camera',
      'variables': {
        'srcdirs': ['.'],
        'manifest': 'frc971_camera.mf',
      },
      'dependencies': [
        '<(AOS)/atom_code/camera/camera.gyp:aos_camera',
        '<(DEPTH)/frc971/queues/queues.gyp:frc971_queues_so',
      ],
      'export_dependent_settings': [
        '<(AOS)/atom_code/camera/camera.gyp:aos_camera',
        '<(DEPTH)/frc971/queues/queues.gyp:frc971_queues_so',
      ],
      'includes': ['../../../aos/build/java.gypi'],
    },
    {
      'target_name': 'frc971',
      'variables': {
        'main_jar': 'frc971_camera',
      },
      'dependencies': [
        'frc971_camera',
      ],
      'includes': ['../../../aos/build/onejar.gypi'],
    },
  ],
}
