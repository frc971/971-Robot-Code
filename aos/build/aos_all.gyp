# This file has the executables etc that AOS builds.
# User .gyp files for the atom should depend on :Atom.
# User .gyp files for the crio should depend on :Crio.
{
  'targets': [
    {
      'target_name': 'Atom',
      'type': 'none',
      'variables': {
        'no_rsync': 1,
      },
      'dependencies': [
        '../atom_code/camera/camera.gyp:CameraHTTPStreamer',
        '../atom_code/camera/camera.gyp:CameraReader',
        '../atom_code/core/core.gyp:*',
        '../atom_code/ipc_lib/ipc_lib.gyp:raw_queue_test',
        '../atom_code/ipc_lib/ipc_lib.gyp:ipc_stress_test',
        '../atom_code/starter/starter.gyp:starter_exe',
        '../atom_code/starter/starter.gyp:netconsole',
        '../common/common.gyp:queue_test',
        '../common/common.gyp:die_test',
        '../common/util/util.gyp:trapezoid_profile_test',
        '../common/glibusb/glibusb.gyp:gbuffer_test',
        '../common/glibusb/glibusb.gyp:glibusb_test',
        'Common',
      ],
    },
    {
      'target_name': 'Crio',
      'type': 'none',
      'dependencies': [
        'Common',
      ],
    },
    {
      'target_name': 'Common',
      'type': 'none',
      'variables': {
        'no_rsync': 1,
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:type_traits_test',
        '<(AOS)/common/common.gyp:time_test',
        '<(AOS)/common/common.gyp:mutex_test',
        '<(AOS)/common/common.gyp:condition_test',
        '<(AOS)/common/common.gyp:once_test',
        '<(AOS)/common/logging/logging.gyp:logging_impl_test',
      ],
    },
  ],
}
