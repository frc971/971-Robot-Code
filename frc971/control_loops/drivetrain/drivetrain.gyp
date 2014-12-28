{
  'targets': [
    {
      'target_name': 'drivetrain_loop',
      'type': 'static_library',
      'sources': ['drivetrain.q'],
      'variables': {
        'header_path': 'frc971/control_loops/drivetrain',
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
      ],
      'dependencies': [
        'drivetrain_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
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
        'drivetrain_loop',
      ],
    },
    {
      'target_name': 'drivetrain_lib_test',
      'type': 'executable',
      'sources': [
        'drivetrain_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'drivetrain_loop',
        'drivetrain_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(AOS)/common/common.gyp:queues',
      ],
    },
    {
      'target_name': 'drivetrain',
      'type': 'executable',
      'sources': [
        'drivetrain_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'drivetrain_lib',
        'drivetrain_loop',
      ],
    },
  ],
}
