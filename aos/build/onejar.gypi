# Include this file in any target that should get packaged with OneJAR.
#
# To use, create a target of the following form:
# {
#   'target_name': 'whatever',
#   'variables': {
#     'main_jar': 'something',
#   },
#   'includes': ['path/to/onejar.gypi'],
# },
# See below for more variables.
{
  'type': 'none',
  'variables': {
# The names of loadable_module targets to add to the jar.
    'jni_libs': [],
# Additional jars to add to the output.
# Named this so that targets from java.gypi will add themselves automatically.
    'classpath': [],
    'jar_dir': '<(PRODUCT_DIR)/jars',
    'create_onejar': '<(AOS)/build/create_onejar',
    'out_onejar': '<(rsync_dir)/<(_target_name).jar',
    'main_jar_file': '<(jar_dir)/<(main_jar).jar',
    'shared_objects': ">!(echo '>(jni_libs)' | sed 's:[^ ]*:<(so_dir)/lib\\0.so:g')",
    'no_rsync': 1,
  },
  'dependencies': [
    '<(EXTERNALS):onejar',
  ],
  'product_dir': '<(PRODUCT_DIR)',
  'actions': [
    {
      'action_name': 'create onejar',
      'message': 'Creating OneJAR jar',
      'inputs': [
        '<(create_onejar)',
        '>@(classpath)',
        '<(main_jar_file)',
        '>@(shared_objects)',
      ],
      'outputs': [
        '<(out_onejar)',
      ],
      'action': [
        '<(create_onejar)',
        '<(main_jar_file)',
        '<(INTERMEDIATE_DIR)', '>(classpath)',
        '<(out_onejar)', '>(onejar_jar)',
        '>(shared_objects)',
      ],
    },
  ],
}
