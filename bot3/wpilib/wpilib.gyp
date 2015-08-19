{
  'targets': [
    {
      'target_name': 'wpilib_interface_bot3',
      'type': 'executable',
      'sources': [
        'wpilib_interface.cc'
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:stl_mutex',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):WPILib',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:hall_effect',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:joystick_sender',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:loop_output_handler',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:buffered_pcm',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:gyro_sender',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:logging_queue',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_lib',
        '<(DEPTH)/bot3/control_loops/elevator/elevator.gyp:elevator_lib',
        '<(DEPTH)/bot3/control_loops/intake/intake.gyp:intake_lib',
      ],
    },
  ],
}
