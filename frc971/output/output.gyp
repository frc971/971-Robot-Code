{
  'targets': [
    {
      'target_name': 'CameraServer',
      'type': 'executable',
      'sources': [
        'CameraServer.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/atom_code/output/output.gyp:http_server',
        '../frc971.gyp:common',
        '<(AOS)/atom_code/atom_code.gyp:init',
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
      'conditions': [
        ['OS=="atom"', {
            'sources': ['AtomMotorWriter.cc'],
            'dependencies': [
              '<(AOS)/atom_code/output/output.gyp:motor_output',
              '<(AOS)/atom_code/messages/messages.gyp:messages',
              '<(AOS)/atom_code/atom_code.gyp:init',
              '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
              '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
              '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
              '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
              '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
            ],
          }, {
            'sources': ['CRIOMotorWriter.cc'],
          }
        ],
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
