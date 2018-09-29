def _single_queue_file_impl(ctx):
  args = [
    '-h_file_path', ctx.outputs.header.path,
    '-cc_file_path', ctx.outputs.cc.path,
    '-src_filename', ctx.file.src.short_path,
    '-I', '.',
    ctx.file.src.path,
  ]
  ctx.action(
    outputs = [
      ctx.outputs.header,
      ctx.outputs.cc,
    ],
    inputs = [ ctx.file.src ] + ctx.attr.q_deps.transitive_q_files,
    executable = ctx.executable._queue_compiler,
    arguments = args,
    mnemonic = 'QGen',
    progress_message = 'Generating C++ code for %s' % ctx.file.src.short_path,
  )

def _single_queue_file_outputs(src):
  return {
    'header': src.name + '.h',
    'cc': src.name + '.cc',
  }

_single_queue_file = rule(
    attrs = {
        "src": attr.label(
            mandatory = True,
            single_file = True,
            allow_files = [".q"],
        ),
        "q_deps": attr.label(
            providers = ["transitive_q_files"],
            mandatory = True,
        ),
        "package_name": attr.string(
            mandatory = True,
        ),
        "_queue_compiler": attr.label(
            executable = True,
            default = Label("//aos/build/queues:compiler"),
            cfg = "host",
        ),
    },
    output_to_genfiles = True,
    outputs = _single_queue_file_outputs,
    implementation = _single_queue_file_impl,
)

def _q_deps_impl(ctx):
  transitive_q_files = ctx.files.srcs
  for dep in ctx.attr.deps:
    transitive_q_files = transitive_q_files + dep.transitive_q_files
  return struct(transitive_q_files = transitive_q_files)

_q_deps = rule(
    attrs = {
        "srcs": attr.label_list(
            mandatory = True,
            non_empty = True,
            allow_files = [".q"],
        ),
        "deps": attr.label_list(
            mandatory = True,
            non_empty = False,
            providers = ["transitive_q_files"],
        ),
    },
    implementation = _q_deps_impl,
)

"""Creates a C++ library from a set of .q files.

Attrs:
  srcs: A list of .q files.
  deps: Other queue_library rules this one depends on.
"""

def queue_library(name, srcs, deps = [],
                  compatible_with = None, restricted_to = None,
                  visibility = None):
  q_deps = _q_deps(
    name = name + '__q_deps',
    srcs = srcs,
    deps = [dep + '__q_deps' for dep in deps],
    visibility = visibility,
    compatible_with = compatible_with,
    restricted_to = restricted_to,
  )

  for src in srcs:
    _single_queue_file(
      name = name + '_' + src,
      src = src,
      q_deps = ':%s__q_deps' % name,
      package_name = PACKAGE_NAME,
      visibility = ['//visibility:private'],
      compatible_with = compatible_with,
      restricted_to = restricted_to,
    )

  native.cc_library(
    name = name,
    srcs = [src + '.cc' for src in srcs],
    hdrs = [src + '.h' for src in srcs],
    deps = deps + [
      '//aos:once',
      '//aos:queues',
      '//aos:queue_types',
      '//aos/logging:printf_formats',
    ],
    visibility = visibility,
    compatible_with = compatible_with,
    restricted_to = restricted_to,
  )
