{
  'targets': [
    {
      'target_name': 'joystick_reader_bot3',
      'type': 'executable',
      'sources': [
        'joystick_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/prime/input/input.gyp:joystick_input',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/actions/actions.gyp:action_lib',

        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(DEPTH)/bot3/control_loops/elevator/elevator.gyp:elevator_lib',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/bot3/control_loops/elevator/elevator.gyp:elevator_queue',
        '<(DEPTH)/bot3/control_loops/intake/intake.gyp:intake_queue',
        '<(DEPTH)/bot3/autonomous/autonomous.gyp:auto_queue',
      ],
    },
  ],
}
