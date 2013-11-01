{
  'targets': [
    {
      'target_name': 'constants',
      'type': 'static_library',
      'sources': [
        'constants.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/network/network.gyp:team_number',
      ],
    }
  ],
}
