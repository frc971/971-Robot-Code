licenses(['notice'])

_header_dirs = [
  'wpilibc/wpilibC++/include',
  'wpilibc/wpilibC++Devices/include',
  'wpilibc/shared/include',
  'wpilibc/Athena/include',
  'hal/include',
  'hal/lib/Athena/FRC_FPGA_ChipObject',
  'hal/lib/Athena',
]

cc_library(
 name = 'wpilib',
 visibility = ['//visibility:public'],
 srcs = glob([
   'wpilibc/Athena/src/*.cpp',
   'wpilibc/Athena/src/Internal/*.cpp',
   'wpilibc/shared/src/**/*.cpp',

   'hal/lib/Athena/*.cpp',
   'hal/lib/Athena/cpp/*.cpp',
   'hal/lib/Athena/ctre/*.cpp',
   'hal/lib/Shared/*.cpp',

   'networktables/ntcore/src/**/*.cpp',
 ]),
 copts = [
   '-Wno-unused-parameter',
   '-Wno-switch-enum',
   '-Wno-attributes',
   '-Wno-cast-align',
   '-Wno-cast-qual',
   '-Wno-deprecated-declarations',
   '-Wno-error',
 ],
 deps = [
   '//third_party/ntcore_2016:ntcore',
   '//external:ni-libraries',
 ],
 hdrs = glob([d + '/**/*.h' for d in _header_dirs]) + glob([d + '/**/*.hpp' for d in _header_dirs]) + ['wpilibc/shared/include/Task.inc'],
 includes = _header_dirs,
 linkopts = [
   '-lpthread',
 ],
 defines = [
   'WPILIB2016=1',
 ],
)
