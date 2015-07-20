{
  'targets': [
    {
      'target_name': 'http_status',
      'type': 'executable',
      'sources': [
        'http_status.cc',
      ],
      'actions': [
        {
          'action_name': 'http_status_gen_embedded',
          'inputs': [
            '<!@(find ./www_defaults)',
            '<(AOS)/externals/seasocks/gen_embedded.py',
          ],
          'outputs': [
            '<(SHARED_INTERMEDIATE_DIR)/http_status/frc971/http_status/embedded.h',
          ],
          'action': [
            'python', '<(AOS)/externals/seasocks/gen_embedded.py', '<(_outputs)',
          ],
        },
      ],
      'include_dirs': [
        '<(SHARED_INTERMEDIATE_DIR)/http_status/'
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):seasocks',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_queue',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/common.gyp:time',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'www',
          ],
        },
      ],
    },
  ],
}
