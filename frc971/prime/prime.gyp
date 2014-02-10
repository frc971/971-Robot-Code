{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../autonomous/autonomous.gyp:auto',
        '../input/input.gyp:joystick_reader',
        '../output/output.gyp:motor_writer',
        '../input/input.gyp:sensor_receiver',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:uart_reader_main',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:test_sensor_receiver',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:packet_finder_test',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:cows_test',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'scripts/start_list.txt',
          ],
        },
      ],
    },
  ],
}
