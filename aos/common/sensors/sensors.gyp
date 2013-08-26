{
  'targets': [
    {
      'target_name': 'sensor_sink',
      'type': 'static_library',
      'sources': [
      ],
      'dependencies': [
        'sensors',
      ],
      'export_dependent_settings': [
        'sensors',
      ],
    },
    {
      'target_name': 'sensors',
      'type': 'static_library',
      'sources': [
        'sensors.cc'
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:controls',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:controls',
      ],
    },
    {
      'target_name': 'sensors_test',
      'type': '<(aos_target)',
      'sources': [
        'sensors_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'sensors',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
    {
      'target_name': 'sensor_receiver',
      'type': 'static_library',
      'sources': [
        #'sensor_receiver-tmpl.h'
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
        'sensors',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:gtest_prod',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
        'sensors',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:gtest_prod',
      ],
    },
    {
      'target_name': 'sensor_receiver_test',
      'type': 'executable',
      'sources': [
        'sensor_receiver_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'sensor_receiver',
        '<(AOS)/common/common.gyp:time',
        'sensors',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
  ],
}
