{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Atom',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_lib_test',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto',
        '<(DEPTH)/frc971/input/input.gyp:JoystickReader',
        '<(DEPTH)/frc971/input/input.gyp:GyroReader',
        #'../input/input.gyp:AutoMode',
        '<(DEPTH)/frc971/output/output.gyp:MotorWriter',
        '<(DEPTH)/frc971/output/output.gyp:CameraServer',
        #'camera/camera.gyp:frc971',
        '<(DEPTH)/frc971/../gyro_board/src/libusb-driver/libusb-driver.gyp:get',
        '<(DEPTH)/frc971/input/input.gyp:gyro_board_reader',
        '<(DEPTH)/frc971/input/input.gyp:gyro_sensor_receiver',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'scripts/aos_module.ko',
            'scripts/start_list.txt',
          ],
        },
      ],
    },
  ],
}
