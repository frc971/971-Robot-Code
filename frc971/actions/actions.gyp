{
  'targets': [
    {
      'target_name': 'shoot_action_queue',
      'type': 'static_library',
      'sources': ['shoot_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
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
      'target_name': 'selfcatch_action_queue',
      'type': 'static_library',
      'sources': ['selfcatch_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
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
	  'target_name': 'action',
      'type': 'static_library',
      'dependencies': [
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/build/aos.gyp:logging',
		],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/build/aos.gyp:logging',
	  ]
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
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
      ],
    },
    {
      'target_name': 'selfcatch_action_lib',
      'type': 'static_library',
      'sources': [
        'selfcatch_action.cc',
      ],
      'dependencies': [
        'selfcatch_action_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
      ],
    },
    {
      'target_name': 'shoot_action',
      'type': 'executable',
      'sources': [
        'shoot_action_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'shoot_action_queue',
        'shoot_action_lib',
		'action',
      ],
    },
    {
      'target_name': 'selfcatch_action',
      'type': 'executable',
      'sources': [
        'selfcatch_action_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'selfcatch_action_queue',
        'selfcatch_action_lib',
		'action',
      ],
    },
  ],
}
