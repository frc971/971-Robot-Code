{
  'targets': [
    {
      'target_name': 'core',
      'type': 'executable',
      'sources': [
        'core.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
    {
      'target_name': 'BinaryLogReader',
      'type': 'executable',
      'sources': [
        'BinaryLogReader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
    {
      'target_name': 'LogStreamer',
      'type': 'executable',
      'sources': [
        'LogStreamer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
    {
      'target_name': 'LogDisplayer',
      'type': 'executable',
      'sources': [
        'LogDisplayer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
    {
      'target_name': 'CRIOLogReader',
      'type': 'executable',
      'sources': [
        'CRIOLogReader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
  ],
}
