{
  'targets': [
    {
      'target_name': 'replay_claw',
      'type': 'executable',
      'variables': {
        'no_rsync': 1,
      },
      'sources': [
        'replay_claw.cc',
      ],
      'dependencies': [
        'claw_queue',
        '<(AOS)/common/controls/controls.gyp:replay_control_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'claw_loop',
      'type': 'static_library',
      'sources': ['claw.q'],
      'variables': {
        'header_path': 'y2014/control_loops/claw',
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
      'target_name': 'claw_lib',
      'type': 'static_library',
      'sources': [
        'claw.cc',
        'claw_motor_plant.cc',
      ],
      'dependencies': [
        'claw_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/y2014/y2014.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:polytope',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:coerce_goal',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/logging/logging.gyp:matrix_logging',
      ],
      'export_dependent_settings': [
        'claw_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:polytope',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:coerce_goal',
      ],
    },
    {
      'target_name': 'claw_lib_test',
      'type': 'executable',
      'sources': [
        'claw_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'claw_loop',
        'claw_lib',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
      ],
    },
    {
      'target_name': 'claw_calibration',
      'type': 'executable',
      'sources': [
        'claw_calibration.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'claw_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/y2014/y2014.gyp:constants',
      ],
    },
    {
      'target_name': 'claw',
      'type': 'executable',
      'sources': [
        'claw_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'claw_lib',
      ],
    },
  ],
}
