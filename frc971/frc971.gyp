{
  'targets': [
    {
      'target_name': 'common',
      'type': 'static_library',
      'sources': [
        'constants.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    }
  ],
}
