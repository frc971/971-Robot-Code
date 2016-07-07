# TODO(austin): I bet this is wrong.
licenses(['restricted'])

load('/tools/build_rules/fortran', 'fortran_library')

# We can't create _wrapper.so in the slycot folder, and can't move it.
# The best way I found to do this is to modify _wrapper.pyf to instead generate
# a _fortranwrapper.so library, and then place a _wrapper.py file in slycot/
# which loads _fortranwrapper from the correct location.  This means that I
# don't need to modify the repository.
genrule(
  name = '_fortranwrapper_pyf',
  srcs = ['slycot/src/_wrapper.pyf'],
  outs = ['slycot/src/_fortranwrapper.pyf'],
  cmd = 'cat $(SRCS) | sed \'s/_wrapper/_fortranwrapper/\' > $(OUTS)'
)

# Now generate the module wrapper.
genrule(
  name = '_fortranwrappermodule',
  srcs = [
    'slycot/src/analysis.pyf',
    'slycot/src/synthesis.pyf',
    'slycot/src/_fortranwrapper.pyf',
    'slycot/src/math.pyf',
    'slycot/src/transform.pyf',
  ],
  outs = ['_fortranwrappermodule.c'],
  cmd = '/usr/bin/python /usr/bin/f2py $(location :slycot/src/_fortranwrapper.pyf) --include-paths external/slycot_repo/slycot/src/ --coutput $(OUTS)',
)

# Build it.
cc_library(
  name = 'slycot_c',
  srcs = [
    ':_fortranwrappermodule',
  ],
  deps = [
    ':fortran_files',
    '@usr_repo//:python2.7_lib',
    '@usr_repo//:python2.7_f2py',
  ],
  copts = [
    '-Wno-error',
    '-Wno-incompatible-pointer-types-discards-qualifiers',
    '-Wno-cast-align',
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
    '-Wno-unused-function',
  ],
)

# Now actually build the fortran files.
fortran_library(
  name = 'fortran_files',
  srcs = glob(['slycot/src/*.f']),
)

# Link it all together.  Make sure it is dynamically linked since I don't know
# how to build the fortran files in statically to a single .so yet, and I'm not
# sure bazel does either.
cc_binary(
  name = '_fortranwrapper.so',
  deps = [
    ':fortran_files',
    ':slycot_c',
  ],
  linkopts = ['-shared', '-lblas', '-llapack'],
  linkstatic = False,
)

# Generate the _wrapper file which loads _fortranwrapper and pretends.
genrule(
  name = '_wrapper',
  outs = ['slycot/_wrapper.py'],
  cmd = 'echo "from external.slycot_repo._fortranwrapper import *" > $(OUTS)',
  output_to_bindir = True,
)

# Now present a python library for slycot
py_library(
  name = 'slycot',
  srcs = [
    'slycot/analysis.py',
    'slycot/examples.py',
    'slycot/__init__.py',
    'slycot/math.py',
    'slycot/synthesis.py',
    'slycot/transform.py',
    ':_wrapper',
  ],
  data = [
    ':_fortranwrapper.so',
  ],
  visibility = ['//visibility:public'],
)
