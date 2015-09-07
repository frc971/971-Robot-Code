package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'logging_queue',
  srcs = [
    'logging.q',
  ],
)

cc_library(
  name = 'encoder_and_potentiometer',
  srcs = [
    'encoder_and_potentiometer.cc',
  ],
  hdrs = [
    'encoder_and_potentiometer.h',
  ],
  deps = [
    '//third_party/WPILib',
    ':dma_edge_counting',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/linux_code/ipc_lib:mutex',
  ],
)

cc_library(
  name = 'dma_edge_counting',
  srcs = [
    'dma_edge_counting.cc',
  ],
  hdrs = [
    'dma_edge_counting.h',
  ],
  deps = [
    '//third_party/WPILib',
    '//aos/common/logging',
    ':hall_effect',
  ],
)

cc_library(
  name = 'interrupt_edge_counting',
  srcs = [
    'interrupt_edge_counting.cc',
  ],
  hdrs = [
    'interrupt_edge_counting.h',
  ],
  deps = [
    '//third_party/WPILib',
    '//aos/common/logging',
    '//aos/common:stl_mutex',
    '//aos/common:time',
    '//aos/linux_code:init',
  ],
)

cc_library(
  name = 'buffered_pcm',
  srcs = [
    'buffered_solenoid.cc',
    'buffered_pcm.cc',
  ],
  hdrs = [
    'buffered_solenoid.h',
    'buffered_pcm.h',
  ],
  deps = [
    '//third_party/WPILib',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'gyro_interface',
  srcs = [
    'gyro_interface.cc',
  ],
  hdrs = [
    'gyro_interface.h',
  ],
  deps = [
    '//third_party/WPILib',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'gyro_sender',
  srcs = [
    'gyro_sender.cc',
  ],
  hdrs = [
    'gyro_sender.h',
  ],
  deps = [
    '//frc971/queues:gyro',
    ':gyro_interface',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/common/util:phased_loop',
    '//aos/common/messages:robot_state',
    '//aos/linux_code:init',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'loop_output_handler',
  srcs = [
    'loop_output_handler.cc',
  ],
  hdrs = [
    'loop_output_handler.h',
  ],
  deps = [
    '//aos/common:scoped_fd',
    '//aos/linux_code:init',
    '//aos/common:time',
    '//aos/common/util:log_interval',
    '//aos/common/messages:robot_state',
  ],
)

cc_library(
  name = 'joystick_sender',
  srcs = [
    'joystick_sender.cc',
  ],
  hdrs = [
    'joystick_sender.h',
  ],
  deps = [
    '//third_party/WPILib',
    '//aos/common/messages:robot_state',
    '//aos/linux_code:init',
    '//aos/common/network:team_number',
    '//aos/common/logging:queue_logging',
  ],
)

cc_library(
  name = 'hall_effect',
  deps = [
    '//third_party/WPILib',
  ],
)
