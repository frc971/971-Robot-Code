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

# Names of WPILib "devices" I don't want to deal with making trivial updates to
# chop out various ugliness or have to vet for sanity.
_excluded_devices = [
  'ADXL345_I2C',
  'GearTooth',
  'Ultrasonic',
  'Victor',
  'TalonSRX',
  'CANTalon',
  'CANJaguar',
  'PIDController',
  'Spark',
  'Servo',
  'VictorSP',
  'SD540',
  'RobotDrive',
  'Jaguar',
  'ADXL345_SPI',
  'DoubleSolenoid',
  'AnalogPotentiometer',
  'ADXL362',
  'AnalogGyro',
  'AnalogAccelerometer',
]

# Whole subdirectories of WPILib we don't want around.
_excluded_shared_directories = [
  'SmartDashboard',
  'LiveWindow',
  'Commands',
  'Buttons',
  'Filters',
]

# Header files we don't want to have.
_bad_hdrs = ([
  'wpilibc/Athena/include/WPILib.h',
] + ['wpilibc/shared/include/%s/**/*' % d
     for d in _excluded_shared_directories] +
 ['wpilibc/*/include/%s.h' % d for d in _excluded_devices])
_h_hdrs = glob([d + '/**/*.h' for d in _header_dirs], exclude=_bad_hdrs)
_hpp_hdrs = glob([d + '/**/*.hpp' for d in _header_dirs], exclude=_bad_hdrs)

cc_library(
 name = 'wpilib',
 visibility = ['//third_party:__pkg__'],
 srcs = glob([
   'wpilibc/Athena/src/*.cpp',
   'wpilibc/Athena/src/Internal/*.cpp',
   'wpilibc/shared/src/**/*.cpp',

   'hal/lib/Athena/*.cpp',
   'hal/lib/Athena/cpp/*.cpp',
   'hal/lib/Athena/ctre/*.cpp',
   'hal/lib/Shared/*.cpp',
 ], exclude=(['wpilibc/shared/src/%s/**/*' % d for d in _excluded_shared_directories] +
             ['wpilibc/*/src/%s.cpp' % d for d in _excluded_devices])),
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
   '//external:ni-libraries-2016',
   '//aos/common/logging',
 ],
 hdrs = _h_hdrs + _hpp_hdrs + ['wpilibc/shared/include/Task.inc'],
 includes = _header_dirs,
 linkopts = [
   '-lpthread',
 ],
 defines = [
   'WPILIB2016=1',
 ],
)
