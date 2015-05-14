{
  'targets': [
    {
      'target_name': 'replay_shooter',
      'type': 'executable',
      'variables': {
        'no_rsync': 1,
      },
      'sources': [
        'replay_shooter.cc',
      ],
      'dependencies': [
        'shooter_queue',
        '<(AOS)/common/controls/controls.gyp:replay_control_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'shooter_queue',
      'type': 'static_library',
      'sources': ['shooter.q'],
      'variables': {
        'header_path': 'y2014/control_loops/shooter',
      },
      'dependencies': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'shooter_lib',
      'type': 'static_library',
      'sources': [
        'shooter.cc',
        'shooter_motor_plant.cc',
        'unaugmented_shooter_motor_plant.cc',
      ],
      'dependencies': [
        'shooter_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/y2014/y2014.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
      ],
      'export_dependent_settings': [
        'shooter_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
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
        'shooter_queue',
        'shooter_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'shooter',
      'type': 'executable',
      'sources': [
        'shooter_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'shooter_lib',
      ],
    },
  ],
}
