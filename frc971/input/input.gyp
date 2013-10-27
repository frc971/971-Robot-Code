{
  'targets': [
    {
      'target_name': 'JoystickReader',
      'type': 'executable',
      'sources': [
        'JoystickReader.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/input/input.gyp:joystick_input',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',

        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/frc971/control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_loop',
        '<(DEPTH)/frc971/control_loops/wrist/wrist.gyp:wrist_loop',
        '<(DEPTH)/frc971/control_loops/index/index.gyp:index_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
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
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        'usb_receiver',
      ],
    },
    {
      'target_name': 'usb_receiver',
      'type': 'static_library',
      'sources': [
        'usb_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:controls',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(AOS)/common/common.gyp:time',
      ],
    },
  ],
}
