{
  'targets': [
    {
      'target_name': 'test_joystick',
      'type': 'executable',
      'sources': [
        'test_joystick.cc',
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
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
