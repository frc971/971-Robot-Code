{
  'targets': [
    {
      'target_name': 'CameraServer',
      'type': 'executable',
      'sources': [
        'CameraServer.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/output/output.gyp:http_server',
        '../frc971.gyp:constants',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
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
  ],
}
