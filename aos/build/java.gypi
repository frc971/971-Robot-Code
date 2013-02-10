# Include this file in any target that is going to build java files.
#
# To use, create a target of the following form:
# {
#   'target_name': 'whatever',
#   'variables': {
#     'srcdirs': ['.', 'java'],
#   },
#   'includes': ['path/to/java.gypi'],
# }
# See below for more variables.
# To make any output jars include some loadable modules, set the 'jni_libs'
#   variable in 'direct_dependent_settings'. Making this easier causes lots of
#   recursion issues in gyp.
#   The dependency on these targets also has to be added manually.
{
  'type': 'none',
  'variables': {
# The manifest file for creating the jar.
    'manifest%': '/dev/null',
# Additional jars/directories to add to the classpath when compiling.
# This target will automatically add itself to this list for any dependents.
    'classpath': [],
# Classes to generate JNI headers for.
# They will be able to be #included as "jni/package_ClassName.h" by targets
#   that depend on this one.
    'gen_headers': [],
# Like 'srcdirs', except not required to exist at gyp time. However, nothing
#   here will depend on any files in these directories.
    'gen_srcdirs': ['/dev/null'],
# Like 'gen_srcdirs', except all folders that are children of this folder will
#   be used instead.
    'gen_srcdir_parents%': [],
    'srcdirs': ['/dev/null'],
    'jar_dir': '<(PRODUCT_DIR)/jars',
    'java_files': '<!(find <(srcdirs) -name *.java)',
    'create_jar': '<(AOS)/build/create_jar',
    'out_jar': '<(jar_dir)/<(_target_name).jar',
    'header_dir': '<(SHARED_INTERMEDIATE_DIR)/jni_headers_<!(pwd | sed s:/:_:g)_<(_target_name)',
    'no_rsync': 1,
  },
  'direct_dependent_settings': {
    'variables': {
      'classpath': ['<(out_jar)'],
    },
    'include_dirs': [
      '<(header_dir)',
    ],
  },
  'actions': [
    {
      'action_name': 'run javac',
      'message': 'Compiling java code',
      'inputs': [
        '<(create_jar)',
        '<@(java_files)',
        '>@(classpath)',
        '>@(gen_srcdirs)',
        '>(manifest)',
      ],
      'outputs': [
        '<(out_jar)',
      ],
      'action': [
        '<(create_jar)',
        '<(srcdirs) <(gen_srcdirs)',
        '<(INTERMEDIATE_DIR)', '>(classpath)',
        '>(gen_srcdir_parents)',
        '>(manifest)', '<(out_jar)',
        '<(header_dir)/jni', '>(gen_headers)',
      ],
    },
  ],
}
