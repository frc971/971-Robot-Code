package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'wpilib_interface_bot3',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:stl_mutex',
    '//aos/common/logging',
    '//aos/externals:wpilib',
    '//bot3/control_loops/drivetrain:drivetrain_queue',
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
    '//bot3/autonomous:auto_queue',
    '//bot3/control_loops/drivetrain:drivetrain_lib',
    '//bot3/control_loops/elevator:elevator_lib',
    '//bot3/control_loops/intake:intake_lib',
  ],
)
