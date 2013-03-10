{
  'targets': [
    {
# This one includes interrupt_bridge.h too.
      'target_name': 'interrupt_notifier',
      'type': 'static_library',
      'sources': [
        'interrupt_bridge.cc',
        'interrupt_bridge_c.c',
        'interrupt_bridge_demo.cc',
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
    {
      'target_name': 'limit_encoder_reader',
      'type': 'static_library',
      'sources': [
        'limit_encoder_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/crio/hardware/hardware.gyp:counter',
        '<(AOS)/crio/hardware/hardware.gyp:digital_source',
        '<(AOS)/common/common.gyp:mutex',
        'interrupt_notifier',
      ],
      'export_dependent_settings': [
        '<(AOS)/crio/hardware/hardware.gyp:counter',
        '<(AOS)/crio/hardware/hardware.gyp:digital_source',
        '<(AOS)/common/common.gyp:mutex',
        'interrupt_notifier',
      ],
    },
  ],
}
