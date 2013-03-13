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
  ],
}
