package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:stl_mutex',
    '//aos/common/logging',
    '//aos/externals:wpilib',
    '//y2014_bot3/control_loops/drivetrain:drivetrain_queue',
    '//aos/common/controls:control_loop',
    '//aos/common/util:log_interval',
    '//aos/common:time',
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common/util:phased_loop',
    '//aos/common/util:wrapping_counter',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:gyro_sender',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:wpilib_robot_base',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:pdp_fetcher',
    '//y2014_bot3/autonomous:auto_queue',
    '//y2014_bot3/control_loops/drivetrain:drivetrain_lib',
    '//y2014_bot3/control_loops/rollers:rollers_lib',
  ],
)
