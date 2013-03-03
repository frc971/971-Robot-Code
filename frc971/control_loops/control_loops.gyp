{
  'variables': {
    'loop_files': [
      'DriveTrain.q',
      'index_motor.q',
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
    {
      'target_name': 'index_lib',
      'type': 'static_library',
      'sources': [
        'index.cc',
        'index_motor_plant.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/frc971.gyp:common',
        'state_feedback_loop',
      ],
    },
    {
      'target_name': 'index_lib_test',
      'type': 'executable',
      'sources': [
        'index_lib_test.cc',
        'transfer_motor_plant.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:libaos',
        'control_loops',
        'index_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        'state_feedback_loop',
      ],
    },
    {
      'target_name': 'index',
      'type': 'executable',
      'sources': [
        'index_main.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        'index_lib',
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
