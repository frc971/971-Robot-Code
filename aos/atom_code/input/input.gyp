{
  'targets': [
    {
      'target_name': 'joystick',
      'type': 'static_library',
      'sources': [
         'JoystickInput.cpp'
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/common/network/network.gyp:socket',
      ]
    },
  ],
}
