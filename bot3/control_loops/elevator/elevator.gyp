{
  'targets': [
    {
      'target_name': 'elevator_queue',
      'type': 'static_library',
      'sources': ['elevator.q'],
      'variables': {
        'header_path': 'bot3/control_loops/elevator',
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
      'target_name': 'elevator_lib',
      'type': 'static_library',
      'sources': [
        'elevator.cc',
        'elevator_motor_plant.cc',
        'integral_elevator_motor_plant.cc',
      ],
      'dependencies': [
        'elevator_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/control_loops/voltage_cap/voltage_cap.gyp:voltage_cap',
      ],
      'export_dependent_settings': [
        'elevator_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'elevator_lib_test',
      'type': 'executable',
      'sources': [
        'elevator_lib_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'elevator_lib',
        '<(DEPTH)/bot3/control_loops/control_loops.gyp:position_sensor_sim',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop_test',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'elevator',
      'type': 'executable',
      'sources': [
        'elevator_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'elevator_lib',
      ],
    },
  ],
}
