{
  'targets': [
    {
      'target_name': 'action_client',
      'type': 'static_library',
      'sources': [
        #'action_client.h',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:queues',
      ],
    },
    {
      'target_name': 'action_queue',
      'type': 'static_library',
      'sources': ['action.q'],
      'variables': {
        'header_path': 'frc971/actions',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'shoot_action_queue',
      'type': 'static_library',
      'sources': ['shoot_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
      },
      'dependencies': [
        'action_queue',
      ],
      'export_dependent_settings': [
        'action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
      },
      'dependencies': [
        'action_queue',
      ],
      'export_dependent_settings': [
        'action_queue',
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
        'action_queue',
      ],
      'export_dependent_settings': [
        'action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'catch_action_queue',
      'type': 'static_library',
      'sources': ['catch_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
      },
      'dependencies': [
        'action_queue',
      ],
      'export_dependent_settings': [
        'action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
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
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        'action_client',
        'action',
      ],
      'export_dependent_settings': [
        'action',
        'shoot_action_queue',
        'action_client',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'drivetrain_action_lib',
      'type': 'static_library',
      'sources': [
        'drivetrain_action.cc',
      ],
      'dependencies': [
        'drivetrain_action_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        'action_client',
        'action',
        '<(EXTERNALS):eigen',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
      ],
      'export_dependent_settings': [
        'action',
        'drivetrain_action_queue',
        'action_client',
      ],
    },
    {
      'target_name': 'action',
      'type': 'static_library',
      'sources': [
        #'action.h',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
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
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
      ],
    },
    {
      'target_name': 'catch_action_lib',
      'type': 'static_library',
      'sources': [
        'catch_action.cc',
      ],
      'dependencies': [
        'catch_action_queue',
        'action',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
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
      'target_name': 'drivetrain_action',
      'type': 'executable',
      'sources': [
        'drivetrain_action_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'drivetrain_action_queue',
        'drivetrain_action_lib',
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
    {
      'target_name': 'catch_action',
      'type': 'executable',
      'sources': [
        'catch_action_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'catch_action_queue',
        'catch_action_lib',
        'action',
      ],
    },
  ],
}
