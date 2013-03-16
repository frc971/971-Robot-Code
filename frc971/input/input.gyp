{
  'targets': [
    {
      'target_name': 'actions',
      'type': 'static_library',
      'sources': ['AutoMode.act'],
      'variables': {
        'header_path': 'frc971/input',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:controls',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'JoystickReader',
      'type': 'executable',
      'sources': [
        'JoystickReader.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/input/input.gyp:joystick',
        'actions',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
    {
      'target_name': 'sensor_unpacker',
      'type': 'static_library',
      'sources': [
        'sensor_unpacker.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
    {
      'target_name': 'sensor_receiver',
      'type': 'executable',
      'sources': [
        'sensor_receiver.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        'sensor_unpacker',
        '<(AOS)/common/sensors/sensors.gyp:sensor_receiver',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
    {
      'target_name': 'sensor_packer',
      'type': 'static_library',
      'sources': [
        'sensor_packer.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/crio/hardware/hardware.gyp:counter',
        '<(AOS)/crio/hardware/hardware.gyp:digital_source',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_lib',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/crio/hardware/hardware.gyp:counter',
        '<(AOS)/crio/hardware/hardware.gyp:digital_source',
      ],
    },
    {
      'target_name': 'GyroReader',
      'type': 'executable',
      'sources': [
        'GyroReader.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'AutoMode',
      'type': 'executable',
      'sources': [
        'AutoMode.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        'actions',
# TODO(brians) this shouldn't need to be here
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
    },
  ],
}
