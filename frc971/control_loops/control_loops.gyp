{
  'variables': {
    'loop_files': [
      'DriveTrain.q',
    ]
  },
  'targets': [
    {
      'target_name': 'control_loops',
      'type': 'static_library',
      'sources': ['<@(loop_files)'],
      'variables': {
        'header_path': 'frc971/control_loops',
      },
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'DriveTrain',
      'type': 'executable',
      'sources': [
        'DriveTrain.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(EXTERNALS):eigen',
      ],
    },
  ],
}
