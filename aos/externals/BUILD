_header_dirs = [
  'allwpilib/wpilibc/wpilibC++/include',
  'allwpilib/wpilibc/wpilibC++Devices/include',
  'allwpilib/hal/include',
  'allwpilib/hal/lib/Athena/FRC_FPGA_ChipObject',
  'allwpilib/hal/lib/Athena',
]

cc_library(
  name = 'wpilib_2015',
  visibility = ['//visibility:public'],
  srcs = glob([
    'allwpilib/wpilibc/wpilibC++/src/*.cpp',
    'allwpilib/wpilibc/wpilibC++Devices/src/*.cpp',
    'allwpilib/wpilibc/wpilibC++Devices/src/Internal/*.cpp',
    'allwpilib/hal/lib/Athena/*.cpp',
    'allwpilib/hal/lib/Athena/ctre/*.cpp',
  ], [
    # This looks like an older version of the file, so we want to use the other
    # one.
    'allwpilib/wpilibc/wpilibC++Devices/include/NetworkCommunication/FRCComm.h',
  ]) + [
    'allwpilib/hal/lib/Athena/NetworkCommunication/FRCComm.h',
    'allwpilib/ni-libraries/libFRC_NetworkCommunication.so.1.5.0',
    'allwpilib/ni-libraries/libRoboRIO_FRC_ChipObject.so.1.2.0',
    'allwpilib/ni-libraries/libNiFpgaLv.so.14.0.0',
    'allwpilib/ni-libraries/libNiFpga.so.14.0.0',
    'allwpilib/ni-libraries/libNiRioSrv.so.14.0.0',
    'allwpilib/ni-libraries/libspi.so.1.0.0',
    'allwpilib/ni-libraries/libi2c.so.1.0.0',
  ],
  defines = [
    'WPILIB2015=1',
  ],
  copts = [
    '-Wno-error',
    '-Wno-unused-parameter',
    '-Wno-switch-enum',
  ],
  hdrs = glob([d + '/**/*.h' for d in _header_dirs] + [d + '/**/*.hpp' for d in _header_dirs]),
  includes = _header_dirs,
  linkopts = [
    '-lpthread',
  ],
)

cc_library(
  name = 'wpilib',
  visibility = ['//visibility:public'],
  deps = [
    ':wpilib_2015',
    #'//third_party/allwpilib_2016:wpilib',
  ],
  includes = [
    'forwpilib',
  ],
  hdrs = [
    'forwpilib/dma.h',
  ],
  srcs = [
    'forwpilib/dma.cc',
  ],
)
