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
        '../control_loops/fridge/fridge.gyp:fridge',
        '../control_loops/fridge/fridge.gyp:fridge_lib_test',
        '../control_loops/fridge/fridge.gyp:replay_fridge',
        '../control_loops/claw/claw.gyp:claw',
        '../control_loops/claw/claw.gyp:claw_lib_test',
        '../control_loops/claw/claw.gyp:replay_claw',
        '../autonomous/autonomous.gyp:auto',
        '../y2015.gyp:joystick_reader',
        '../http_status/http_status.gyp:http_status',
        '../util/util.gyp:kinematics_test',
        '../actors/actors.gyp:binaries',
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
  ],
}
