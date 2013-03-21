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
        #'../atom_code/async_action:*', # TODO(brians) fix this broken test
        '../atom_code/ipc_lib/ipc_lib.gyp:*',
        '../atom_code/starter/starter.gyp:*',
        '../crio/crio.gyp:unsafe_queue_test',
        '../common/common.gyp:queue_test',
        #'../common/messages/messages.gyp:*', # TODO(brians) did this test ever exist?
        '../common/common.gyp:die_test',
        '../common/util/util.gyp:trapezoid_profile_test',
        '../common/sensors/sensors.gyp:sensor_receiver_test',
        'Common',
        # TODO(brians): move this to Common
        '<(AOS)/common/sensors/sensors.gyp:sensors_test',
      ],
    },
    {
      'target_name': 'Crio',
      'type': 'none',
      'dependencies': [
        '../crio/googletest/googletest.gyp:*',
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
        '<(AOS)/common/common.gyp:once_test',
        '<(AOS)/common/logging/logging.gyp:logging_impl_test',
      ],
    },
  ],
}
