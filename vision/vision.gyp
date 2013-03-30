{
  'targets': [
    {
      'target_name': 'OpenCVWorkTask',
      'type': 'executable',
      'sources': [
        'OpenCVWorkTask.cpp',
        'CameraProcessor.cpp',
        'BinaryServer.cpp',
        'PacketNotifier.cpp',
        'JPEGRoutines.cpp',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):libjpeg',
        '<(EXTERNALS):opencv',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
    {
      'target_name': 'GoalMaster',
      'type': 'executable',
      'sources': [
        'GoalMaster.cpp',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
