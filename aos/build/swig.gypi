# Include this file in any target that needs to use swig wrappers.
#
# To use, create a target of the following form:
# {
#   'target_name': 'my_target_javawrap',
#   'type': 'static_library', # or any other type that can handle .cc files
#   'sources': [
#     'aos/example/target.swig',
#   ],
#   'variables': {
#     'package': 'aos.test',
#   },
#   'includes': ['path/to/swig.gypi'],
# },
# Code that depends on this target will be able to use the swig wrapped
# java classes.
#
# using <http://src.chromium.org/svn/trunk/src/build/protoc.gypi> as an
# example of how this should work
{
  'variables': {
    'prefix_dir': '<(SHARED_INTERMEDIATE_DIR)/',
    'out_dir': '<(prefix_dir)/<(_target_name)/',
    'output_java_wrap': '<(out_dir)/<(RULE_INPUT_ROOT)_wrap.cc',
    'java_dir': '<(out_dir)/<(RULE_INPUT_ROOT)_java',
    'no_rsync': 1,
  },
  'rules': [
    {
      'rule_name': 'genswig',
      'extension': 'swig',
      'outputs': [
        '<(output_java_wrap)',
        '<(java_dir)',
      ],
      'action': [
        '<(DEPTH)/aos/build/mkdirswig',
        '<(java_dir)',
        '-I<(DEPTH)',
        '-outdir', ' <(java_dir)',
        '-package', '<(package)',
        '-o', '<(output_java_wrap)',
        '-c++',
        '-Wall',
        '-Wextra',
        '-java',
        '<(RULE_INPUT_PATH)'],
      'message': 'Generating C++ code from <(RULE_INPUT_DIRNAME)/<(RULE_INPUT_ROOT).swig',
      'process_outputs_as_sources': 1,
    },
  ],
  'cflags': [
# For the swig-generated C++ code.
    '-fno-strict-aliasing',
    '-Wno-cast-qual',
  ],
  'include_dirs': [
    '<(prefix_dir)/<(_target_name)',
    '/usr/lib/jvm/default-java/include',
    '/usr/lib/jvm/default-java/include/linux',
  ],
  'direct_dependent_settings': {
    'include_dirs': [
      '<(prefix_dir)/<(_target_name)',
      '/usr/lib/jvm/default-java/include',
      '/usr/lib/jvm/default-java/include/linux',
    ],
    'variables': {
      'gen_srcdir_parents': ['<(out_dir)'],
    },
  },
  'hard_dependency': 1,
}
