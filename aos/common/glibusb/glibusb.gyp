{
  'targets': [
    {
      'target_name': 'gbuffer',
      'type': 'static_library',
      'sources': [
        'gbuffer.cc',
        'ghexdump.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'gbuffer_test',
      'type': 'executable',
      'sources': [
        'gbuffer_test.cc',
      ],
      'dependencies': [
        'gbuffer',
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
  ],
}
