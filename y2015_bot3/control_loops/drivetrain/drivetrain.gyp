{
  'targets': [
    {
      'target_name': 'replay_drivetrain_bot3',
      'type': 'executable',
      'variables': {
        'no_rsync': 1,
      },
      'sources': [
        'replay_drivetrain.cc',
      ],
      'dependencies': [
        'drivetrain_queue',
        '<(AOS)/common/controls/controls.gyp:replay_control_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'drivetrain_queue',
      'type': 'static_library',
      'sources': ['drivetrain.q'],
      'variables': {
        'header_path': 'bot3/control_loops/drivetrain',
      },
      'dependencies': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'polydrivetrain_plants',
      'type': 'static_library',
      'sources': [
        'polydrivetrain_dog_motor_plant.cc',
        'drivetrain_dog_motor_plant.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'drivetrain_lib',
      'type': 'static_library',
      'sources': [
        'drivetrain.cc',
        'polydrivetrain_cim_plant.cc',
        'drivetrain_dog_motor_plant.cc',
        'polydrivetrain_dog_motor_plant.cc',
      ],
      'dependencies': [
        'drivetrain_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/controls/controls.gyp:polytope',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:coerce_goal',
        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/logging/logging.gyp:matrix_logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:polytope',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:coerce_goal',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        'drivetrain_queue',
      ],
    },
    {
      'target_name': 'drivetrain_lib_test_bot3',
      'type': 'executable',
      'sources': [
        'drivetrain_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'drivetrain_queue',
        'drivetrain_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/network/network.gyp:team_number',
      ],
    },
    {
      'target_name': 'drivetrain_bot3',
      'type': 'executable',
      'sources': [
        'drivetrain_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'drivetrain_lib',
        'drivetrain_queue',
      ],
    },
  ],
}
