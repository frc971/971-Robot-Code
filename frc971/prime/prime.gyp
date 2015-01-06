{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',

        '../control_loops/control_loops.gyp:state_feedback_loop_test',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../autonomous/autonomous.gyp:auto',
        '../actions/actions.gyp:drivetrain_action',
        '../frc971.gyp:joystick_reader',
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
        ['ARCHITECTURE=="arm"', {
          'dependencies': [
            '../wpilib/wpilib.gyp:wpilib_interface',
          ],
        }],
      ],
    },
  ],
}
