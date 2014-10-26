{
  'targets': [
    {
      'target_name': 'auto_queue',
      'type': 'static_library',
      'sources': [
        '<(DEPTH)/frc971/autonomous/auto.q',
      ],
      'variables': {
        'header_path': 'frc971/autonomous',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'auto_lib',
      'type': 'static_library',
      'sources': [
        'auto.cc',
      ],
      'dependencies': [
        'auto_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
    },
    {
      'target_name': 'auto',
      'type': 'executable',
      'sources': [
        'auto_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'auto_queue',
        'auto_lib',
      ],
    },
  ],
}
