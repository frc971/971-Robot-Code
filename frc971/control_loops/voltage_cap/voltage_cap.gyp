{
  'targets': [
    {
      'target_name': 'voltage_cap',
      'type': 'static_library',
      'sources': [
        'voltage_cap.cc',
      ],
    },
    {
      'target_name': 'voltage_cap_test',
      'type': 'executable',
      'sources': [
        'voltage_cap_test.cc',
      ],
      'dependencies': [
        'voltage_cap',
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
  ],
}
