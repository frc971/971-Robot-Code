{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:all_tests',

        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../control_loops/rollers/rollers.gyp:rollers',
        '../autonomous/autonomous.gyp:auto',
        '../actions/actions.gyp:drivetrain_action',
        '../input/input.gyp:joystick_reader',
        '../output/output.gyp:motor_writer',
        '../input/input.gyp:sensor_receiver',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:uart_reader_main',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:test_sensor_receiver',
        '<(DEPTH)/bbb_cape/src/flasher/flasher.gyp:stm32_flasher',
        '<(AOS)/prime/input/input.gyp:joystick_proxy',
      ],
      'variables': {
        'cape_src': '<(DEPTH)/bbb_cape/src/cape',
        'cape_hex': '<(cape_src)/.obj/main_comp.hex',
      },
      'actions': [
        {
          'action_name': 'make_cape',
          'inputs': [
            '<!@(find <(cape_src) -name ".*" -prune -o -type f -print)',
            '<(cape_src)/Makefile',
          ],
          'outputs': [
            '<(cape_hex)',
          ],
          'action': ['make', '-C', '<(cape_src)'],
          'message': 'Building cape code',
        },
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            '<(cape_hex)',
            'start_list.txt',
          ],
        },
      ],
    },
  ],
}
