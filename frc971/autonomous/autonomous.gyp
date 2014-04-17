{
  'targets': [
    {
      'target_name': 'auto_queue',
      'type': 'static_library',
      'sources': ['auto.q'],
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
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/actions/actions.gyp:action_client',
        '<(DEPTH)/frc971/actions/actions.gyp:shoot_action_lib',
        '<(DEPTH)/frc971/actions/actions.gyp:drivetrain_action_lib',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/queues/queues.gyp:hot_goal',
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
