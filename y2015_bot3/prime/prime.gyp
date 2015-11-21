{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '../../frc971/frc971.gyp:All',

        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop_test',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:position_sensor_sim_test',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_bot3',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test_bot3',
        '../control_loops/drivetrain/drivetrain.gyp:replay_drivetrain_bot3',
        '../control_loops/intake/intake.gyp:intake',
        '../control_loops/intake/intake.gyp:intake_lib_test',
        '../bot3.gyp:joystick_reader_bot3',
        '../control_loops/elevator/elevator.gyp:elevator',
        '../control_loops/elevator/elevator.gyp:elevator_lib_test',
        #'../control_loops/elevator/elevator.gyp:replay_elevator',
        '../autonomous/autonomous.gyp:auto_bot3',
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
