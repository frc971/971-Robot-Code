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
        '../frc971.gyp:constants',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
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
        'AtomMotorWriter.cc'
      ],
      'dependencies': [
        '<(AOS)/atom_code/output/output.gyp:motor_output',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
