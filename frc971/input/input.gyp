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
      'target_name': 'SensorReader',
      'type': '<(aos_target)',
      'sources': [
        'SensorReader.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/input/input.gyp:sensor_input',
        '<(AOS)/build/aos.gyp:aos_core',
      ],
      'conditions': [
        ['OS!="crio"', {
          'dependencies': [
            '<(AOS)/atom_code/atom_code.gyp:init',
          ],
        }],
      ],
    },
    {
      'target_name': 'SensorWriter',
      'type': '<(aos_target)',
      'sources': [
        'SensorWriter.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:control_loops',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
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
