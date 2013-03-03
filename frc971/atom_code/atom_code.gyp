{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Atom',
        '../control_loops/control_loops.gyp:DriveTrain',
        '../control_loops/wrist/wrist.gyp:wrist',
        '../control_loops/wrist/wrist.gyp:wrist_lib_test',
        '../control_loops/control_loops.gyp:hall_effect_lib_test',
        '../control_loops/control_loops.gyp:angle_adjust_lib_test',
        '../control_loops/control_loops.gyp:angle_adjust',
        '../control_loops/control_loops.gyp:wrist',
        '../control_loops/control_loops.gyp:wrist_lib_test',
        '../input/input.gyp:JoystickReader',
        '../input/input.gyp:SensorReader',
        '../input/input.gyp:GyroReader',
        '../input/input.gyp:AutoMode',
        '../output/output.gyp:MotorWriter',
        '../output/output.gyp:CameraServer',
        'camera/camera.gyp:frc971',
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
