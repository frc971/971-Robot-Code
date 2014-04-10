{
  'targets': [
    {
      'target_name': 'led_setter',
      'type': 'executable',
      'sources': [
        'led_setter.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:led',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'CameraServer',
      'type': 'executable',
      'sources': [
        'CameraServer.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/output/output.gyp:http_server',
        '../frc971.gyp:constants',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'robot.html.tpl',
          ],
        },
      ],
    },
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
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/common/controls/controls.gyp:output_check',
      ],
    },
  ],
}
