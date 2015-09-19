package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'OpenCVWorkTask',
  srcs = [
    'OpenCVWorkTask.cpp',
    'CameraProcessor.cpp',
    'JPEGRoutines.cpp',
    'OpenCVWorkTask.h',
    'CameraProcessor.h',
    'JPEGRoutines.h',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:time',
    '//third_party/libevent',
    '//third_party/libjpeg',
    '//third_party/opencv',
    '//aos/linux_code/camera:buffers',
    '//frc971/queues:queues',
  ],
)

cc_binary(
  name = 'GoalMaster',
  srcs = [
    'GoalMaster.cpp',
    'SensorProcessor.cpp',
    'SensorProcessor.h',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:time',
    '//frc971/queues:queues',
    '//aos/common/logging',
  ],
)
