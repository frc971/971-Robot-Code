{
  'targets': [
    {
      'target_name': 'common',
      'type': 'static_library',
      'sources': [
        'constants.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/output/output.gyp:motor_output',
        '<(AOS)/common/messages/messages.gyp:aos_queues',
      ],
    }
  ],
}
