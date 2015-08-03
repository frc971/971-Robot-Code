{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '../../frc971/frc971.gyp:All',

        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../control_loops/drivetrain/drivetrain.gyp:replay_drivetrain',
        '../control_loops/claw/claw.gyp:claw',
        '../control_loops/claw/claw.gyp:claw_calibration',
        '../control_loops/claw/claw.gyp:claw_lib_test',
        '../control_loops/claw/claw.gyp:replay_claw',
        '../control_loops/shooter/shooter.gyp:shooter',
        '../control_loops/shooter/shooter.gyp:shooter_lib_test',
        '../control_loops/shooter/shooter.gyp:replay_shooter',
        '../autonomous/autonomous.gyp:auto',
        '../y2014.gyp:joystick_reader',
        '../actors/actors.gyp:binaries',
        'hot_goal_reader',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'start_list.txt',
          ],
        },
      ],
      'conditions': [
        ['ARCHITECTURE=="arm_frc"', {
          'dependencies': [
            '../wpilib/wpilib.gyp:wpilib_interface',
          ],
        }],
      ],
    },
    {
      'target_name': 'hot_goal_reader',
      'type': 'executable',
      'sources': [
        'hot_goal_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(DEPTH)/y2014/queues/queues.gyp:hot_goal'
      ],
    },
  ],
}
