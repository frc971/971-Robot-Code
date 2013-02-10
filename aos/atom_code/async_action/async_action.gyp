{
  'targets': [
    {
      'target_name': 'AsyncAction_test',
      'type': 'executable',
      'sources': [
        'AsyncACtion_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
      ],
  ],
},
