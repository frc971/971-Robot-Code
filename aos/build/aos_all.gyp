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
        '<(AOS)/linux_code/linux_code.gyp:core',
        '<(AOS)/linux_code/logging/logging.gyp:binary_log_writer',
        '<(AOS)/linux_code/logging/logging.gyp:log_streamer',
        '<(AOS)/linux_code/logging/logging.gyp:log_displayer',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:raw_queue_test',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:ipc_stress_test',
        '<(AOS)/linux_code/starter/starter.gyp:starter_exe',
        '<(AOS)/linux_code/starter/starter.gyp:netconsole',
        '<(AOS)/common/common.gyp:queue_test',
        '<(AOS)/common/common.gyp:die_test',
        '<(AOS)/common/common.gyp:queue_types_test',
        '<(AOS)/common/util/util.gyp:string_to_num_test',
        '<(AOS)/common/util/util.gyp:trapezoid_profile_test',
        '<(AOS)/common/util/util.gyp:wrapping_counter_test',
        '<(AOS)/common/libc/libc.gyp:dirname_test',
        '<(AOS)/common/libc/libc.gyp:aos_strerror_test',
        '<(AOS)/common/libc/libc.gyp:aos_strsignal_test',
        '<(AOS)/common/util/util.gyp:run_command_test',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:cows_test',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:packet_finder_test',
        '<(AOS)/linux_code/linux_code.gyp:complex_thread_local_test',
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
        '<(AOS)/common/util/util.gyp:options_test',
      ],
    },
  ],
}
