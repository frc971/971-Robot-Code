{
  'targets': [
    {
      'target_name': 'replay_fridge',
      'type': 'executable',
      'variables': {
        'no_rsync': 1,
      },
      'sources': [
        'replay_fridge.cc',
      ],
      'dependencies': [
        'fridge_queue',
        '<(AOS)/common/controls/controls.gyp:replay_control_loop',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
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
        '<(DEPTH)/frc971/zeroing/zeroing.gyp:zeroing',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
        '<(DEPTH)/frc971/zeroing/zeroing.gyp:zeroing',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'fridge_lib',
      'type': 'static_library',
      'sources': [
        'fridge.cc',
        'integral_arm_plant.cc',
        'elevator_motor_plant.cc',
      ],
      'dependencies': [
        'fridge_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/control_loops/voltage_cap/voltage_cap.gyp:voltage_cap',
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
        'arm_motor_plant.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'fridge_lib',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(AOS)/common/common.gyp:time',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:position_sensor_sim',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:team_number_test_environment',
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
