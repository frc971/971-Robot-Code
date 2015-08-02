{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Prime',

        'control_loops/control_loops.gyp:state_feedback_loop_test',
        'control_loops/control_loops.gyp:position_sensor_sim_test',
        'zeroing/zeroing.gyp:zeroing_test',
        'control_loops/voltage_cap/voltage_cap.gyp:voltage_cap_test',
      ],
    },
  ],
}
