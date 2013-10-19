{
  'targets': [
    {
      'target_name': 'CameraServer',
      'type': 'executable',
      'sources': [
        'CameraServer.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/output/output.gyp:http_server',
        '<(DEPTH)/frc971/frc971.gyp:common',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/atom_code/atom_code.gyp:configuration',
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
      'target_name': 'MotorWriter',
      'type': '<(aos_target)',
      'sources': [
        'atom_motor_writer.cc'
      ],
      'dependencies': [
        '<(AOS)/atom_code/output/output.gyp:motor_output',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
