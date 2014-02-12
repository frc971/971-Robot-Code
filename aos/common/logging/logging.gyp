{
  'targets': [
    {
      'target_name': 'logging_impl_test',
      'type': '<(aos_target)',
      'sources': [
        'logging_impl_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'queue_logging',
      'type': 'static_library',
      'sources': [
        'queue_logging.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:queue_types',
      ],
    },
  ],
}
