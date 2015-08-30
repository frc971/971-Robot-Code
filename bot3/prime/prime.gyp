{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',

        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:position_sensor_sim_test',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_bot3',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test_bot3',
        '../control_loops/drivetrain/drivetrain.gyp:replay_drivetrain_bot3',
        '../control_loops/intake/intake.gyp:intake',
        '../control_loops/intake/intake.gyp:intake_lib_test',
        #'../autonomous/autonomous.gyp:auto_bot3',
        '../bot3.gyp:joystick_reader_bot3',
        '<(DEPTH)/frc971/zeroing/zeroing.gyp:zeroing_test',
        #'../http_status/http_status.gyp:http_status',
        '<(DEPTH)/frc971/control_loops/voltage_cap/voltage_cap.gyp:voltage_cap_test',
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
            '../wpilib/wpilib.gyp:wpilib_interface_bot3',
          ],
        }],
      ],
    },
  ],
}
