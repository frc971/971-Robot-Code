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
            '<!@(find ./web)',
          ],
          'outputs': [
            'embedded.h',
          ],
          'action': [
            'python', '<(AOS)/externals/seasocks/gen_embedded.py', 'http_status', '<(_outputs)',
          ],
        },
      ],
      'cflags': [
        # TODO(comran): Fix these once we start writing our own code for the
        # server
        '-Wno-unused-parameter',
        '-Wno-format-nonliteral',
        '-Wno-error=cast-align',
        '-Wno-switch-enum',
        '-Wno-cast-qual',
        '-Wno-strict-aliasing',
        '-Wno-error=strict-aliasing',
      ],
      'dependencies': [
        '<(EXTERNALS):seasocks',
      ],
    },
  ],
}
