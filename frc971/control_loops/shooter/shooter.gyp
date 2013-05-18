{
  'targets': [
    {
      'target_name': 'shooter_loop',
      'type': 'static_library',
      'sources': ['shooter_motor.q'],
      'variables': {
        'header_path': 'frc971/control_loops/shooter',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'shooter_lib',
      'type': 'static_library',
      'sources': [
        'shooter.cc',
        'shooter_motor_plant.cc',
      ],
      'dependencies': [
        'shooter_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/common.gyp:controls',
        'shooter_loop',
      ],
    },
    {
      'target_name': 'shooter_lib_test',
      'type': 'executable',
      'sources': [
        'shooter_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'shooter_loop',
        'shooter_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'shooter_csv',
      'type': 'executable',
      'sources': [
        'shooter_csv.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
        'shooter_loop',
      ],
    },
    {
      'target_name': 'shooter',
      'type': 'executable',
      'sources': [
        'shooter_main.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        'shooter_lib',
        'shooter_loop',
      ],
    },
  ],
}
