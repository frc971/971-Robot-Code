{
  'targets': [
    {
      'target_name': 'atom_logging_test',
      'type': 'executable',
      'sources': [
        'atom_logging_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
  ],
}
