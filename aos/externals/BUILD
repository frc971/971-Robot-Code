_header_dirs = [
  'forwpilib',
  'allwpilib/wpilibc/wpilibC++/include',
  'allwpilib/wpilibc/wpilibC++Devices/include',
  'allwpilib/hal/include',
  'allwpilib/hal/lib/Athena/FRC_FPGA_ChipObject',
  'allwpilib/hal/lib/Athena',
]

cc_library(
  name = 'wpilib',
  visibility = ['//visibility:public'],
  srcs = glob([
    'allwpilib/wpilibc/wpilibC++/src/*.cpp',
    'allwpilib/wpilibc/wpilibC++Devices/src/*.cpp',
    'allwpilib/wpilibc/wpilibC++Devices/src/Internal/*.cpp',
    'allwpilib/hal/lib/Athena/*.cpp',
    'allwpilib/hal/lib/Athena/ctre/*.cpp',
  ]) + [
    'forwpilib/dma.cc',
  ],
  hdrs = glob([d + '/*.h' for d in _header_dirs]),
  includes = _header_dirs,
  linkopts = [
    '-Lallwpilib/ni-libraries',
    '-lpthread',
    '-lFRC_NetworkCommunication',
    '-lRoboRIO_FRC_ChipObject',
    '-lNiFpgaLv',
    '-lNiFpga',
    '-lNiRioSrv',
    '-lspi',
    '-li2c',
  ],
)
