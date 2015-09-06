{
  'targets': [
    {
      'target_name': 'auto_lib',
      'type': 'static_library',
      'sources': [
        'auto.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/y2015/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/y2015/y2015.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/y2015/actors/actors.gyp:drivetrain_action_lib',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(DEPTH)/y2015/control_loops/claw/claw.gyp:claw_queue',
        '<(DEPTH)/y2015/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(DEPTH)/y2015/actors/actors.gyp:stack_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:held_to_lift_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:pickup_action_lib',
        ':auto_queue',
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
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        'auto_lib',
      ],
    },
    {
      'target_name': 'auto_queue',
      'type': 'static_library',
      'sources': ['auto.q'],
      'variables': {
        'header_path': 'y2015/autonomous',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
