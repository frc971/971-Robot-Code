{
  'targets': [
    {
      'target_name': 'joystick_reader',
      'type': 'executable',
      'sources': [
        'joystick_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/prime/input/input.gyp:joystick_input',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',

        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
      ],
    },
    {
      'target_name': 'sensor_receiver',
      'type': 'executable',
      'sources': [
        'sensor_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:sensor_reader',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
      ],
    },
  ],
}
