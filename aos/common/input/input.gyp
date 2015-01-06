{
  'targets': [
    {
      'target_name': 'driver_station_data',
      'type': 'static_library',
      'sources': [
        'driver_station_data.cc',
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
      ],
    },
  ],
}
