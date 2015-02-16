{
  'targets': [
    {
      'target_name': 'team_number',
      'type': 'static_library',
      'sources': [
        'team_number.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:configuration',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:string_to_num',
      ],
    },
    {
      'target_name': 'socket',
      'type': 'static_library',
      'sources': [
        'receive_socket.cc',
        'send_socket.cc',
        'socket.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:inet_addr',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
      ],
    },
  ],
}
