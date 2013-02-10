{
  'targets': [
    {
      'target_name': 'CameraServer',
      'type': 'executable',
      'sources': [
        'CameraServer.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/atom_code/output/output.gyp:http_server',
        '../frc971.gyp:common',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'robot.html.tpl',
          ],
        },
      ],
    },
    {
      'target_name': 'MotorWriter',
      'type': '<(aos_target)',
      'conditions': [
        ['OS=="atom"', {
            'sources': ['AtomMotorWriter.cc'],
            'dependencies': [
              '../frc971.gyp:common',
              '<(AOS)/atom_code/output/output.gyp:motor_output',
              '<(AOS)/atom_code/messages/messages.gyp:messages',
            ],
          }, {
            'sources': ['CRIOMotorWriter.cc'],
          }
        ],
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
    {
      'target_name': 'SensorSender',
      'type': '<(aos_target)',
      'sources': [
        'SensorSender.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
  ],
}
