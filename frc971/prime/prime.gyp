{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',

        '../control_loops/control_loops.gyp:state_feedback_loop_test',
        '../control_loops/control_loops.gyp:position_sensor_sim_test',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../control_loops/fridge/fridge.gyp:fridge',
        '../control_loops/fridge/fridge.gyp:fridge_lib_test',
        '../control_loops/claw/claw.gyp:claw',
        '../control_loops/claw/claw.gyp:claw_lib_test',
        '../autonomous/autonomous.gyp:auto',
        '../frc971.gyp:joystick_reader',
        '../zeroing/zeroing.gyp:zeroing_test',
        '../control_loops/voltage_cap/voltage_cap.gyp:voltage_cap_test',
        '../../aos/common/actions/actions.gyp:action_test',
        '../actors/actors.gyp:drivetrain_action',
        '../actors/actors.gyp:fridge_profile_action',
        '../actors/actors.gyp:fridge_profile_action_test',
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
