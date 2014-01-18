{
  'targets': [
    {
      'target_name': 'wrist_loops',
      'type': 'static_library',
      'sources': ['wrists.q'],
      'variables': {
        'header_path': 'frc971/control_loops/wrists',
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
      'target_name': 'wrists_lib',
      'type': 'static_library',
      'sources': [
        'wrists.cc',
        'top_wrist_motor_plant.cc',
        'top_wrist_motor_plant.cc',
      ],
      'dependencies': [
        'wrist_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        'wrist_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'wrists_lib_test',
      'type': 'executable',
      'sources': [
        'wrists_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'wrist_loops',
        'wrists_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'wrists',
      'type': 'executable',
      'sources': [
        'wrists_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'wrists_lib',
      ],
    },
  ],
}
