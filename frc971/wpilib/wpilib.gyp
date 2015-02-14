{
  'targets': [
    {
      'target_name': 'wpilib_interface',
      'type': 'executable',
      'sources': [
        'wpilib_interface.cc'
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:stl_mutex',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):WPILib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        'hall_effect',
        'joystick_sender',
        'loop_output_handler',
        'buffered_pcm',
        'gyro_sender',
        'dma_edge_counting',
        'interrupt_edge_counting',
        'encoder_and_potentiometer',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
    },
    {
      'target_name': 'encoder_and_potentiometer',
      'type': 'static_library',
      'sources': [
        'encoder_and_potentiometer.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        'dma_edge_counting',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:mutex',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        'dma_edge_counting',
        '<(AOS)/common/common.gyp:mutex',
      ],
    },
    {
      'target_name': 'dma_edge_counting',
      'type': 'static_library',
      'sources': [
        'dma_edge_counting.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
        'hall_effect',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        'hall_effect',
      ],
    },
    {
      'target_name': 'interrupt_edge_counting',
      'type': 'static_library',
      'sources': [
        'interrupt_edge_counting.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:stl_mutex',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:stl_mutex',
      ],
    },
    {
      'target_name': 'buffered_pcm',
      'type': 'static_library',
      'sources': [
        'buffered_solenoid.cc',
        'buffered_pcm.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
    {
      'target_name': 'gyro_interface',
      'type': 'static_library',
      'sources': [
        'gyro_interface.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
    {
      'target_name': 'gyro_sender',
      'type': 'static_library',
      'sources': [
        'gyro_sender.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        'gyro_interface',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        'gyro_interface',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'loop_output_handler',
      'type': 'static_library',
      'sources': [
        'loop_output_handler.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:scoped_fd',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/messages/messages.gyp:robot_state',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:scoped_fd',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
      ],
    },
    {
      'target_name': 'joystick_sender',
      'type': 'static_library',
      'sources': [
        'joystick_sender.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/network/network.gyp:team_number',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
      ],
    },
    {
      'target_name': 'hall_effect',
      'type': 'static_library',
      'sources': [
        #'hall_effect.h',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
  ],
}
