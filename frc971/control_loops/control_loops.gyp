{
  'variables': {
    'loop_files': [
      'DriveTrain.q',
      'angle_adjust_motor.q',
    ]
  },
  'targets': [
    {
      'target_name': 'state_feedback_loop',
      'type': 'static_library',
      'sources': [
        #'state_feedback_loop.h'
        #'StateFeedbackLoop.h'
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
      ],
    },
    {
      'target_name': 'control_loops',
      'type': 'static_library',
      'sources': ['<@(loop_files)'],
      'variables': {
        'header_path': 'frc971/control_loops',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:control_loop_queues',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'hall_effect_lib_test',
      'type': 'executable',
      'sources': [
        'hall_effect_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'angle_adjust_lib',
      'type': 'static_library',
      'sources': [
        'angle_adjust.cc',
        'angle_adjust_motor_plant.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        'state_feedback_loop',
      ],
      'export_dependent_settings': [
        'state_feedback_loop',
        '<(AOS)/common/common.gyp:controls',
        'control_loops',
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
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        'angle_adjust_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
    {
      'target_name': 'angle_adjust',
      'type': 'executable',
      'sources': [
        'angle_adjust_main.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'angle_adjust_lib',
        'control_loops',
      ],
    },
    {
      'target_name': 'DriveTrain',
      'type': 'executable',
      'sources': [
        'DriveTrain.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:controls',
        'control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/atom_code/atom_code.gyp:init',
        'state_feedback_loop',
      ],
    },
  ],
}
