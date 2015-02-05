{
  'targets': [
    {
      'target_name': 'fridge_queue',
      'type': 'static_library',
      'sources': ['fridge.q'],
      'variables': {
        'header_path': 'frc971/control_loops/fridge',
      },
      'dependencies': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',

      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'fridge_lib',
      'type': 'static_library',
      'sources': [
        'fridge.cc',
        'arm_motor_plant.cc',
        'elevator_motor_plant.cc',
      ],
      'dependencies': [
        'fridge_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
      'export_dependent_settings': [
        'fridge_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'fridge_lib_test',
      'type': 'executable',
      'sources': [
        'fridge_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'fridge_lib',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
      ],
    },
    {
      'target_name': 'fridge',
      'type': 'executable',
      'sources': [
        'fridge_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'fridge_lib',
      ],
    },
  ],
}
