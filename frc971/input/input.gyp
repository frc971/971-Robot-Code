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
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
      ],
    },
    {
      'target_name': 'sensor_unpacker',
      'type': 'static_library',
      'sources': [
        'sensor_unpacker.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/common/network/network.gyp:socket',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
      ],
    },
    {
      'target_name': 'gyro_sensor_receiver',
      'type': 'executable',
      'sources': [
        'gyro_sensor_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(EXTERNALS):libusb',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/sensors/sensors.gyp:sensor_receiver',
      ],
    },
    {
      'target_name': 'gyro_board_reader',
      'type': 'executable',
      'sources': [
        'gyro_board_reader.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(EXTERNALS):libusb',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:timing',
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
  ],
}
