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
      'target_name': 'drivetrain_lib',
      'type': 'static_library',
      'sources': [
        'drivetrain.cc',
        'drivetrain_motor_plant.cc',
        'polydrivetrain_motor_plant.cc',
        'polydrivetrain_cim_plant.cc',
      ],
      'dependencies': [
        'drivetrain_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        '<(DEPTH)/aos/build/externals.gyp:libcdd',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/aos/build/externals.gyp:libcdd',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/common.gyp:controls',
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
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
    {
      'target_name': 'drivetrain',
      'type': 'executable',
      'sources': [
        'drivetrain_main.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        'drivetrain_lib',
        'drivetrain_loop',
      ],
    },
  ],
}
