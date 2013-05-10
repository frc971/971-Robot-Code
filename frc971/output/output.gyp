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
        '../frc971.gyp:common',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/messages/messages.gyp:aos_queues',
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
              '<(AOS)/atom_code/atom_code.gyp:init',
              '<(AOS)/build/aos.gyp:logging',
              '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
              '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
              '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
              '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
              '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
            ],
          }, {
            'sources': ['CRIOMotorWriter.cc'],
            'dependencies': [
              '<(EXTERNALS):WPILib',
              '<(AOS)/build/aos.gyp:aos_core',
            ],
          }
        ],
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
