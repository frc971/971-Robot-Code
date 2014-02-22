# Include this file in any target that needs to use files generated from queue
#   etc. definitions.
#
# To use, create a target of the following form:
# {
#   'target_name': 'my_queues',
#   'type': 'static_library', # or any other type that can handle .cc files
#   'sources': [
#     'aos/example/Queue.q',
#     'aos/example/ControlLoop.q',
#   ],
#   'variables': {
#     'header_path': 'aos/example',
#   },
#   'includes': ['path/to/queues.gypi'],
# },
# Code that depends on this target will be able to #include
#   "aos/example/Queue.q.h" and "aos/example/ControlLoop.q.h".
#
# using <http://src.chromium.org/svn/trunk/src/build/protoc.gypi> as an
# example of how this should work
{
  'variables': {
    #'header_path': '>!(python -c "import os.path; print os.path.relpath(\'<(RULE_INPUT_PATH)\', \'<(DEPTH)\')")',
    'prefix_dir': '<(SHARED_INTERMEDIATE_DIR)/<!(echo <(header_path) | sed "s/[^A-Za-z0-9]/_/g")',
    'out_dir': '<(prefix_dir)/<(_target_name)/<(header_path)',
    'gen_namespace%': '>!(echo >(header_path) | sed "s:\([^/]*\).*:\\1:g")',
    'output_h': '<(out_dir)/<(RULE_INPUT_ROOT).q.h',
    'output_cc': '<(out_dir)/<(RULE_INPUT_ROOT).q.cc',
    'output_main': '<(out_dir)/<(RULE_INPUT_ROOT)_main.cc',
    'no_rsync': 1,
  },
  'rules': [
    {
      'variables': {
        'script': '<(AOS)/build/queues/compiler.rb',
      },
      'rule_name': 'genqueue',
      'extension': 'q',
      'outputs': [
        '<(output_h)',
        '<(output_cc)',
      ],
      'inputs': [
        '<(script)',
        '<!@(find <(AOS)/build/queues/ -name *.rb)',
        '<(AOS)/common/queue.h',
        '<(AOS)/common/time.h',
      ],
      'action': ['ruby', '<(script)',
        '-I', '<(DEPTH)',
        '<(RULE_INPUT_PATH)',
        '-cpp_out',
        '<(header_path)',
        '-cpp_base',
        '<(prefix_dir)/<(_target_name)'],
      'message': 'Generating C++ code from <(RULE_INPUT_DIRNAME)/<(RULE_INPUT_ROOT).q',
      'process_outputs_as_sources': 1,
    },
  ],
  'include_dirs': [
    '<(prefix_dir)/<(_target_name)',
  ],
  'direct_dependent_settings': {
    'include_dirs': [
      '<(prefix_dir)/<(_target_name)',
    ],
    'variables': {
      'gen_srcdir_parents': ['<(out_dir)'],
    },
  },
  'dependencies': [
    '<(AOS)/common/common.gyp:queues',
    '<(AOS)/common/common.gyp:once',
    '<(AOS)/common/common.gyp:queue_types',
  ],
  'export_dependent_settings': [
    '<(AOS)/common/common.gyp:queues',
  ],
  'hard_dependency': 1,
}
