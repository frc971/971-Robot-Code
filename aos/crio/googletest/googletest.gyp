{
  'targets': [
    {
      'target_name': 'googletest',
      'type': 'static_library',
      'sources': [
        'google_test_server.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
      ],
    },
  ],
}
