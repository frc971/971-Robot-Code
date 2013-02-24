{
  'variables': {
    'loop_files': [
      'DriveTrain.q',
      'wrist_motor.q',
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
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'wrist_lib',
      'type': 'static_library',
      'sources': [
        'wrist.cc',
        'wrist_motor_plant.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        '<(EXTERNALS):eigen',
      ],
    },
    {
      'target_name': 'wrist_lib_test',
      'type': 'executable',
      'sources': [
        'wrist_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        'wrist_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(EXTERNALS):eigen',
      ],
    },
    {
      'target_name': 'wrist',
      'type': 'executable',
      'sources': [
        'wrist_main.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'wrist_lib',
        'control_loops',
      ],
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
        'control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(EXTERNALS):eigen',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
  ],
}
