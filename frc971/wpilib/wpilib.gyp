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
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):WPILib_roboRIO',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/controls/controls.gyp:sensor_generation',
        '<(AOS)/common/util/util.gyp:log_interval',
        '../frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/common/controls/controls.gyp:output_check',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        'hall_effect',
        'joystick_sender',
        'loop_output_handler',
        'buffered_pcm',
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
        '<(EXTERNALS):WPILib_roboRIO',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib_roboRIO',
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
        '<(EXTERNALS):WPILib_roboRIO',
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
        '<(EXTERNALS):WPILib_roboRIO',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib_roboRIO',
      ],
    },
  ],
}
