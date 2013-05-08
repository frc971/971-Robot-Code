{
  'targets': [
    {
      'target_name': 'driver_station_data',
      'type': 'static_library',
      'sources': [
        'driver_station_data.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib-NetworkRobotValues',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib-NetworkRobotValues',
      ],
    },
  ],
}
