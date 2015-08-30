{
  'targets': [
    {
      'target_name': 'intake_queue',
      'type': 'static_library',
      'sources': ['intake.q'],
      'variables': {
        'header_path': 'bot3/control_loops/intake',
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
      'target_name': 'intake_lib',
      'type': 'static_library',
      'sources': [
        'intake.cc',
      ],
      'dependencies': [
        'intake_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        'intake_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
    },
    {
      'target_name': 'intake_lib_test',
      'type': 'executable',
      'sources': [
        'intake_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'intake_lib',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(AOS)/common/common.gyp:time',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:team_number_test_environment',
      ],
    },
    {
      'target_name': 'intake',
      'type': 'executable',
      'sources': [
        'intake_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'intake_lib',
      ],
    },
  ],
}
