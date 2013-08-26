{
  'targets': [
    {
      'target_name': 'core',
      'type': 'executable',
      'sources': [
        'core.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
    {
      'target_name': 'BinaryLogReader',
      'type': 'executable',
      'sources': [
        'BinaryLogReader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/atom_code/atom_code.gyp:configuration',
      ],
    },
    {
      'target_name': 'LogStreamer',
      'type': 'executable',
      'sources': [
        'LogStreamer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'LogDisplayer',
      'type': 'executable',
      'sources': [
        'LogDisplayer.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
    {
      'target_name': 'CRIOLogReader',
      'type': 'executable',
      'sources': [
        'CRIOLogReader.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
  ],
}
