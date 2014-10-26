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
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/bot3/autonomous/autonomous.gyp:auto_queue',
      ],
    },
    {
      'target_name': 'sensor_receiver',
      'type': 'executable',
      'sources': [
        'sensor_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_constants',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/bot3/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:sensor_reader',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/controls/controls.gyp:output_check',
      ],
    },
  ],
}
