{
  'targets': [
    {
      'target_name': 'netconsole',
      'type': 'executable',
      'sources': [
        'netconsole.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
        '<(AOS)/common/util/util.gyp:inet_addr',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
    {
      'target_name': 'starter_exe',
      'type': 'executable',
      'sources': [
        'starter.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(EXTERNALS):libevent',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/libc/libc.gyp:aos_strsignal',
        '<(AOS)/common/util/util.gyp:run_command',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'starter_loop.sh',
          ],
          'conditions': [
            ['FULL_COMPILER=="gcc_frc"', {
              'files': [
                'starter_roborio.sh',
              ],
            }, {
              'files': [
                'starter.sh',
              ],
            }
          ]],
        },
      ],
    },
  ],
}
