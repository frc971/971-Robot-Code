{
  'targets': [
    {
      'target_name': 'motor_writer',
      'type': 'executable',
      'sources': [
        'motor_writer.cc'
      ],
      'dependencies': [
        '<(AOS)/prime/output/output.gyp:motor_output',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/bot3/queues/queues.gyp:queues',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/controls/controls.gyp:output_check',
      ],
    },
  ],
}
