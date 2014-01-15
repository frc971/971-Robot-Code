{
  'targets': [
    {
      'target_name': 'OpenCVWorkTask',
      'type': 'executable',
      'sources': [
        'OpenCVWorkTask.cpp',
        'CameraProcessor.cpp',
        #'BinaryServer.cpp',
        #'PacketNotifier.cpp',
        'JPEGRoutines.cpp',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):libjpeg',
        '<(EXTERNALS):opencv',
        '<(AOS)/linux_code/camera/camera.gyp:buffers',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
    {
      'target_name': 'GoalMaster',
      'type': 'executable',
      'sources': [
        'GoalMaster.cpp',
        'SensorProcessor.cpp',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/common.gyp:time',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
