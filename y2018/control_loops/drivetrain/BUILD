load('//aos/build:queues.bzl', 'queue_library')

genrule(
  name = 'genrule_drivetrain',
  cmd = '$(location //y2018/control_loops/python:drivetrain) $(OUTS)',
  tools = [
    '//y2018/control_loops/python:drivetrain',
  ],
  outs = [
    'drivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.h',
    'kalman_drivetrain_motor_plant.cc',
  ],
)

genrule(
  name = 'genrule_polydrivetrain',
  cmd = '$(location //y2018/control_loops/python:polydrivetrain) $(OUTS)',
  tools = [
    '//y2018/control_loops/python:polydrivetrain',
  ],
  outs = [
    'polydrivetrain_dog_motor_plant.h',
    'polydrivetrain_dog_motor_plant.cc',
    'polydrivetrain_cim_plant.h',
    'polydrivetrain_cim_plant.cc',
  ],
)

cc_library(
  name = 'polydrivetrain_plants',
  visibility = ['//visibility:public'],
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.cc',
  ],
  hdrs = [
    'polydrivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
    'kalman_drivetrain_motor_plant.h',
  ],
  deps = [
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'drivetrain_base',
  visibility = ['//visibility:public'],
  srcs = [
    'drivetrain_base.cc',
  ],
  hdrs = [
    'drivetrain_base.h',
  ],
  deps = [
    ':polydrivetrain_plants',
    '//frc971/control_loops/drivetrain:drivetrain_config',
    '//frc971:shifter_hall_effect',
  ],
)

cc_binary(
  name = 'drivetrain',
  visibility = ['//visibility:public'],
  srcs = [
    'drivetrain_main.cc',
  ],
  deps = [
    ':drivetrain_base',
    '//aos/linux_code:init',
    '//frc971/control_loops/drivetrain:drivetrain_lib',
  ],
)
