def _single_fortran_object_impl(ctx):
  toolchain_cflags = (ctx.fragments.cpp.compiler_options([]) +
      ctx.fragments.cpp.c_options +
      ctx.fragments.cpp.unfiltered_compiler_options([]) +
      ['-fPIC', '-Wno-maybe-uninitialized', '-Wno-unused-dummy-argument',
       '-Wno-conversion', '-Wno-unused-variable', '-Wno-character-truncation'])

  cmd = toolchain_cflags + ['-c', ctx.file.src.path, '-o', ctx.outputs.pic_o.path]
  filtered_cmd = []
  # Strip out the C/C++/Clang specific flags.
  exclude_flags = ['-fcolor-diagnostics',
                   '-Wswitch-enum',
                   '-Wpointer-arith',
                   '-Wcast-qual',
                   '-Wwrite-strings',
                   '-Wsign-compare',
                   '-Wformat=2',
                   '-Werror',
                   '-Wextra',
                   '-Wno-builtin-macro-redefined',
                   '-Wunused-local-typedefs',
                   '-D__has_feature(x)=0',
                   '-fmacro-backtrace-limit=0']

  for flag in cmd:
    if flag not in exclude_flags and not (flag.startswith('-fsanitize') or
                                          flag.startswith('-fno-sanitize')):
      filtered_cmd.append(flag)

  ctx.action(
    inputs = [ctx.file.src] + ctx.files._cc_toolchain,
    outputs = [ctx.outputs.pic_o],
    mnemonic = "Fortran",
    executable = ctx.fragments.cpp.compiler_executable,
    arguments = filtered_cmd,
    progress_message = 'Building %s' % ctx.outputs.pic_o.short_path,
  )

def _define_fortran_output(src):
  if not src.name.endswith('.f'):
    fail('Fortran files must end in \'.f\'', 'src')

  fortran_file_base = src.name[:-2]
  return {
    'pic_o': fortran_file_base + '.pic.o',
  }


_single_fortran_object = rule(
  implementation = _single_fortran_object_impl,
  attrs = {
    'src': attr.label(single_file=True, allow_files=FileType(['.f'])),
    'cc_libs': attr.label_list(providers=['cc']),
    # TODO(Brian): Replace this with something more fine-grained from the
    # configuration fragment or something.
    '_cc_toolchain': attr.label(
      default = Label('@//tools/cpp:toolchain'),
    ),
  },
  outputs = _define_fortran_output,
  fragments = [
    'cpp',
  ],
)

def fortran_library(name, srcs, deps = [], visibility = None):
  """Builds a shared library from a set of fortran files.

  Args:
    srcs: list of fortran files ending in .f
    deps: cc_library or fortran_library dependencies.
  """
  pic_o_files = []
  for src in srcs:
    pic_o_file = src[:-2] + '.pic.o'
    _single_fortran_object(
      name = name + '_' + pic_o_file,
      src = src,
      visibility = ['//visibility:private'],
    )
    pic_o_files.append(pic_o_file)

  native.cc_library(
    name = name,
    deps = deps,
    srcs = pic_o_files,
    linkopts = [
      '-lgfortran',
    ],
    visibility = visibility,
  )
