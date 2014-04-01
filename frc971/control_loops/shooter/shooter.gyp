{
  'targets': [
    {
      'target_name': 'shooter_loop',
      'type': 'static_library',
      'sources': ['shooter.q'],
      'variables': {
        'header_path': 'frc971/control_loops/shooter',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:control_loop_queues',
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
        'shooter_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(DEPTH)/frc971/queues/queues.gyp:output_check',
      ],
      'export_dependent_settings': [
        'shooter_loop',
        '<(AOS)/common/common.gyp:controls',
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
        'shooter_loop',
        'shooter_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
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
