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
        '<(DEPTH)/y2014/y2014.gyp:constants',
        '<(DEPTH)/y2014/queues/queues.gyp:auto_mode',
        '<(DEPTH)/y2014/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/y2014/control_loops/shooter/shooter.gyp:shooter_queue',
        '<(DEPTH)/y2014/control_loops/claw/claw.gyp:claw_queue',
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
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:dma_edge_counting',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:interrupt_edge_counting',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:encoder_and_potentiometer',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
        '<(DEPTH)/frc971/wpilib/wpilib.gyp:logging_queue',
      ],
    },
  ],
}
