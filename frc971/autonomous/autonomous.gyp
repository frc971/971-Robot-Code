{
  'targets': [
    {
      'target_name': 'auto_queue',
      'type': 'static_library',
      'sources': ['auto.q'],
      'variables': {
        'header_path': 'frc971/autonomous',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'shoot_action_queue',
      'type': 'static_library',
      'sources': ['shoot_action.q'],
      'variables': {
        'header_path': 'frc971/autonomous',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
      ],
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
        'shoot_action_lib',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:controls',
      ],
    },
    {
      'target_name': 'shoot_action_lib',
      'type': 'static_library',
      'sources': [
        'shoot_action.cc',
      ],
      'dependencies': [
        'shoot_action_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
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
