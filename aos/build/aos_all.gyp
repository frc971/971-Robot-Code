# This file has the executables etc that AOS builds.
# User .gyp files for the prime should depend on :Prime.
# User .gyp files for the crio should depend on :Crio.
{
  'targets': [
    {
      'target_name': 'Prime',
      'type': 'none',
      'variables': {
        'no_rsync': 1,
      },
      'dependencies': [
        '../linux_code/linux_code.gyp:core',
        '../linux_code/logging/logging.gyp:binary_log_writer',
        '../linux_code/logging/logging.gyp:log_streamer',
        '../linux_code/logging/logging.gyp:log_displayer',
        '../linux_code/ipc_lib/ipc_lib.gyp:raw_queue_test',
        '../linux_code/ipc_lib/ipc_lib.gyp:ipc_stress_test',
        '../linux_code/starter/starter.gyp:starter_exe',
        '../linux_code/starter/starter.gyp:netconsole',
        '../common/common.gyp:queue_test',
        '../common/common.gyp:die_test',
        '../common/common.gyp:queue_types_test',
        '../common/util/util.gyp:trapezoid_profile_test',
        '../common/util/util.gyp:wrapping_counter_test',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:cows_test',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:packet_finder_test',
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
