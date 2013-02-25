{
  'targets': [
    {
      'target_name': 'limit_encoder_reader',
      'type': 'static_library',
      'sources': [
        'limit_encoder_reader.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        'interrupt_notifier',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
    {
      'target_name': 'ByteBuffer',
      'type': 'static_library',
      'sources': [
        # 'ByteBuffer.h',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
    {
      'target_name': 'interrupt_notifier',
      'type': 'static_library',
      'sources': [
        'interrupt_bridge.cc',
        'interrupt_bridge_c.c',
        'interrupt_bridge_demo.cc',
        # 'interrupt_notifier-tmpl.h',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/messages/messages.gyp:aos_queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):WPILib',
      ],
    },
  ],
}
