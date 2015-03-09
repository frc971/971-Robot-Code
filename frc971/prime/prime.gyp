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
        '../control_loops/drivetrain/drivetrain.gyp:replay_drivetrain',
        '../control_loops/fridge/fridge.gyp:fridge',
        '../control_loops/fridge/fridge.gyp:fridge_lib_test',
        '../control_loops/fridge/fridge.gyp:replay_fridge',
        '../control_loops/claw/claw.gyp:claw',
        '../control_loops/claw/claw.gyp:claw_lib_test',
        '../control_loops/claw/claw.gyp:replay_claw',
        '../autonomous/autonomous.gyp:auto',
        '../frc971.gyp:joystick_reader',
        '../zeroing/zeroing.gyp:zeroing_test',
        '../http_status/http_status.gyp:http_status',
        '../control_loops/voltage_cap/voltage_cap.gyp:voltage_cap_test',
        '../../aos/common/actions/actions.gyp:action_test',
        '../actors/actors.gyp:drivetrain_action',
        '../actors/actors.gyp:claw_action',
        '../actors/actors.gyp:score_action',
        '../actors/actors.gyp:score_action_test',
        '../actors/actors.gyp:pickup_action',
        '../actors/actors.gyp:stack_action',
        '../actors/actors.gyp:stack_and_lift_action',
        '../actors/actors.gyp:stack_and_hold_action',
        '../actors/actors.gyp:held_to_lift_action',
        '../actors/actors.gyp:can_pickup_action',
        '../actors/actors.gyp:horizontal_can_pickup_action',
        '../actors/actors.gyp:lift_action',
        '../actors/actors.gyp:claw_action_test',
        '../actors/actors.gyp:stack_action_test',
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
