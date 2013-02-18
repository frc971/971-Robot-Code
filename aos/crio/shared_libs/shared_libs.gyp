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
  ],
}
