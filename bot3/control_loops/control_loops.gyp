{
  'targets': [
    {
      'target_name': 'position_sensor_sim',
      'type': 'static_library',
      'sources': ['position_sensor_sim.cc'],
      'dependencies': [
        '<(DEPTH)/bot3/control_loops/elevator/elevator.gyp:elevator_queue',
      ],
    },
  ],
}
