{
  'targets': [
    {
      'target_name': 'wrist_loop',
      'type': 'static_library',
      'sources': ['wrist_motor.q'],
      'variables': {
        'header_path': 'frc971/control_loops/wrist',
      },
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'wrist_lib',
      'type': 'static_library',
      'sources': [
        'wrist.cc',
        'wrist_motor_plant.cc',
        'unaugmented_wrist_motor_plant.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'wrist_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/common.gyp:controls',
        'wrist_loop',
      ],
    },
    {
      'target_name': 'wrist_lib_test',
      'type': 'executable',
      'sources': [
        'wrist_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'wrist_loop',
        'wrist_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'wrist',
      'type': 'executable',
      'sources': [
        'wrist_main.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        'wrist_lib',
        'wrist_loop',
      ],
    },
  ],
}
