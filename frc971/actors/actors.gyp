{
  'targets': [
    {
      'target_name': 'binaries',
      'type': 'none',
      'dependencies': [
        'drivetrain_action',
        'score_action',
        'score_action_test',
        'pickup_action',
        'stack_action',
        'stack_and_lift_action',
        'stack_and_hold_action',
        'held_to_lift_action',
        'can_pickup_action',
        'horizontal_can_pickup_action',
        'lift_action',
        'stack_action_test',
      ],
    },
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'drivetrain_action_lib',
      'type': 'static_library',
      'sources': [
        'drivetrain_actor.cc',
      ],
      'dependencies': [
        'drivetrain_action_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
      ],
    },
    {
      'target_name': 'drivetrain_action',
      'type': 'executable',
      'sources': [
        'drivetrain_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
        'drivetrain_action_lib',
      ],
    },
    {
      'target_name' : 'fridge_profile_lib',
      'type' : 'static_library',
      'sources' : [
        'fridge_profile_lib.cc',
      ],
      'dependencies' : [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings' : [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
      ],
    },
    {
      'target_name': 'score_action_queue',
      'type': 'static_library',
      'sources': ['score_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'score_action_lib',
      'type': 'static_library',
      'sources': [
        'score_actor.cc',
      ],
      'dependencies': [
        'score_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(EXTERNALS):eigen',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'score_action_queue',
        '<(EXTERNALS):eigen',
      ],
    },
    {
      'target_name': 'score_action',
      'type': 'executable',
      'sources': [
        'score_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'score_action_queue',
        'score_action_lib',
      ],
    },
    {
      'target_name': 'score_action_test',
      'type': 'executable',
      'sources': [
        'score_actor_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:team_number_test_environment',
        'score_action_queue',
        'score_action_lib',
      ],
    },
    {
      'target_name': 'pickup_action_queue',
      'type': 'static_library',
      'sources': ['pickup_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'pickup_action_lib',
      'type': 'static_library',
      'sources': [
        'pickup_actor.cc',
      ],
      'dependencies': [
        'pickup_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'pickup_action_queue',
      ],
    },
    {
      'target_name': 'pickup_action',
      'type': 'executable',
      'sources': [
        'pickup_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'pickup_action_queue',
        'pickup_action_lib',
      ],
    },
    {
      'target_name': 'can_pickup_action_queue',
      'type': 'static_library',
      'sources': ['can_pickup_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'can_pickup_action_lib',
      'type': 'static_library',
      'sources': [
        'can_pickup_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'can_pickup_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        'fridge_profile_lib',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'can_pickup_action_queue',
      ],
    },
    {
      'target_name': 'can_pickup_action',
      'type': 'executable',
      'sources': [
        'can_pickup_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'can_pickup_action_queue',
        'can_pickup_action_lib',
      ],
    },
    {
      'target_name': 'horizontal_can_pickup_action_queue',
      'type': 'static_library',
      'sources': ['horizontal_can_pickup_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'horizontal_can_pickup_action_lib',
      'type': 'static_library',
      'sources': [
        'horizontal_can_pickup_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'horizontal_can_pickup_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        'fridge_profile_lib',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'horizontal_can_pickup_action_queue',
      ],
    },
    {
      'target_name': 'horizontal_can_pickup_action',
      'type': 'executable',
      'sources': [
        'horizontal_can_pickup_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'horizontal_can_pickup_action_queue',
        'horizontal_can_pickup_action_lib',
      ],
    },
    {
      'target_name': 'held_to_lift_action_queue',
      'type': 'static_library',
      'sources': ['held_to_lift_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'lift_action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'lift_action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'held_to_lift_action_lib',
      'type': 'static_library',
      'sources': [
        'held_to_lift_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'held_to_lift_action_queue',
        'lift_action_lib',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'held_to_lift_action_queue',
        'fridge_profile_lib',
      ],
    },
    {
      'target_name': 'held_to_lift_action',
      'type': 'executable',
      'sources': [
        'held_to_lift_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'held_to_lift_action_queue',
        'held_to_lift_action_lib',
      ],
    },
    {
      'target_name': 'stack_and_hold_action_queue',
      'type': 'static_library',
      'sources': ['stack_and_hold_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'stack_action_queue',
        'lift_action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'stack_action_queue',
        'lift_action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'stack_and_hold_action_lib',
      'type': 'static_library',
      'sources': [
        'stack_and_hold_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'stack_and_hold_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_and_hold_action_queue',
        'fridge_profile_lib',
      ],
    },
    {
      'target_name': 'stack_and_hold_action',
      'type': 'executable',
      'sources': [
        'stack_and_hold_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_and_hold_action_queue',
        'stack_and_hold_action_lib',
      ],
    },
    {
      'target_name': 'stack_and_lift_action_queue',
      'type': 'static_library',
      'sources': ['stack_and_lift_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'stack_action_queue',
        'lift_action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
        'stack_action_queue',
        'lift_action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'stack_and_lift_action_lib',
      'type': 'static_library',
      'sources': [
        'stack_and_lift_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'stack_and_lift_action_queue',
        'stack_action_lib',
        'lift_action_lib',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_and_lift_action_queue',
        'fridge_profile_lib',
      ],
    },
    {
      'target_name': 'stack_and_lift_action',
      'type': 'executable',
      'sources': [
        'stack_and_lift_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_and_lift_action_queue',
        'stack_and_lift_action_lib',
      ],
    },
    {
      'target_name': 'stack_action_queue',
      'type': 'static_library',
      'sources': [
        'stack_action.q',
        'stack_action_params.q',
      ],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'stack_action_test',
      'type': 'executable',
      'sources': [
        'stack_actor_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:team_number_test_environment',
        'stack_action_queue',
        'stack_action_lib',
      ],
    },
    {
      'target_name': 'stack_action_lib',
      'type': 'static_library',
      'sources': [
        'stack_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'stack_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
      ],
      'export_dependent_settings': [
        'fridge_profile_lib',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_action_queue',
      ],
    },
    {
      'target_name': 'stack_action',
      'type': 'executable',
      'sources': [
        'stack_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'stack_action_queue',
        'stack_action_lib',
      ],
    },
    {
      'target_name': 'lift_action_queue',
      'type': 'static_library',
      'sources': [
        'lift_action.q',
        'lift_action_params.q',
      ],
      'variables': {
        'header_path': 'frc971/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'lift_action_lib',
      'type': 'static_library',
      'sources': [
        'lift_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_lib',
        'lift_action_queue',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
      ],
      'export_dependent_settings': [
        'fridge_profile_lib',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'lift_action_queue',
      ],
    },
    {
      'target_name': 'lift_action',
      'type': 'executable',
      'sources': [
        'lift_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'lift_action_queue',
        'lift_action_lib',
      ],
    },
  ],
}
