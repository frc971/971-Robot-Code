{
  'targets': [
    {
      'target_name': 'angle_adjust_loop',
      'type': 'static_library',
      'sources': ['angle_adjust_motor.q'],
      'variables': {
        'header_path': 'frc971/control_loops/angle_adjust',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:control_loop_queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:control_loop_queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'angle_adjust_lib',
      'type': 'static_library',
      'sources': [
        'angle_adjust.cc',
        'angle_adjust_motor_plant.cc',
        'unaugmented_angle_adjust_motor_plant.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        'angle_adjust_loop',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/common.gyp:controls',
        'angle_adjust_loop',
      ],
    },
    {
      'target_name': 'angle_adjust_lib_test',
      'type': 'executable',
      'sources': [
        'angle_adjust_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'angle_adjust_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        'angle_adjust_loop',
      ],
    },
    {
      'target_name': 'angle_adjust_csv',
      'type': 'executable',
      'sources': [
        'angle_adjust_csv.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
        'angle_adjust_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:queues',
      ],
    },
    {
      'target_name': 'angle_adjust',
      'type': 'executable',
      'sources': [
        'angle_adjust_main.cc',
      ],
      'dependencies': [
        'angle_adjust_lib',
        'angle_adjust_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
  ],
}
