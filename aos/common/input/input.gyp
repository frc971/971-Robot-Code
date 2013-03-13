{
  'targets': [
    {
      'target_name': 'sensor_input',
      'type': 'static_library',
      'sources': [
        #'SensorInput-tmpl.h',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
