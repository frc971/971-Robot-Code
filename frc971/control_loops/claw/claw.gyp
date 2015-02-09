{
  'targets': [
    {
      'target_name': 'claw_queue',
      'type': 'static_library',
      'sources': ['claw.q'],
      'variables': {
        'header_path': 'frc971/control_loops/claw',
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
        'claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        'claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
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
        'claw_lib',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:position_sensor_sim',
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
