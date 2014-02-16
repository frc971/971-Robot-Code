{
  'targets': [
    {
      'target_name': 'claw_loops',
      'type': 'static_library',
      'sources': ['claw.q'],
      'variables': {
        'header_path': 'frc971/control_loops/claw',
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
      'target_name': 'claw_lib',
      'type': 'static_library',
      'sources': [
        'claw.cc',
        'top_claw_motor_plant.cc',
        'bottom_claw_motor_plant.cc',
        'unaugmented_top_claw_motor_plant.cc',
        'unaugmented_bottom_claw_motor_plant.cc',
      ],
      'dependencies': [
        'claw_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        'claw_loops',
        '<(AOS)/common/common.gyp:controls',
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
        'claw_loops',
        'claw_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
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
