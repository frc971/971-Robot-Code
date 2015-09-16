ZIP_PATH = '/usr/bin/zip'

ruby_file_types = FileType(['.rb'])

def _collect_transitive_sources(ctx):
  source_files = set(order='compile')
  for dep in ctx.attr.deps:
    source_files += dep.transitive_ruby_files

  source_files += ruby_file_types.filter(ctx.files.srcs)
  return source_files

def _ruby_library_impl(ctx):
  transitive_sources = _collect_transitive_sources(ctx)
  return struct(
    transitive_ruby_files = transitive_sources,
  )

def _ruby_binary_impl(ctx):
  main_file = ctx.files.srcs[0]
  transitive_sources = _collect_transitive_sources(ctx)
  executable = ctx.outputs.executable
  manifest_file = ctx.outputs.manifest

  path_map = '\n'.join([('%s|%s' % (item.short_path, item.path))
                        for item in transitive_sources])
  ctx.file_action(output=manifest_file, content=path_map)

  ctx.action(
    inputs = list(transitive_sources) + [manifest_file],
    outputs = [ executable ],
    arguments = [ manifest_file.path, executable.path, main_file.path ],
    executable = ctx.executable._ruby_linker,
  )

  return struct(
    files = set([executable]),
    runfiles = ctx.runfiles(collect_data = True),
  )


_ruby_attrs = {
  'srcs': attr.label_list(
    allow_files = ruby_file_types,
    mandatory = True,
    non_empty = True,
  ),
  'deps': attr.label_list(
    providers = ['transitive_ruby_files'],
    allow_files = False,
  ),
  'data': attr.label_list(
    allow_files = True,
    cfg = DATA_CFG,
  ),
}

'''Packages ruby code into a library.

The files can use require with paths from the base of the workspace and/or
require_relative with other files that are part of the library.
require also works from the filesystem as usual.

Attrs:
  srcs: Ruby source files to include.
  deps: Other ruby_library rules to include.
'''
ruby_library = rule(
  implementation = _ruby_library_impl,
  attrs = _ruby_attrs,
)

'''Packages ruby code into a binary which can be run.

See ruby_library for details on how require works.

Attrs:
  srcs: Ruby source files to include. The first one is loaded to at startup.
  deps: ruby_library rules to include.
'''
ruby_binary = rule(
  implementation = _ruby_binary_impl,
  executable = True,
  attrs = _ruby_attrs + {
    '_ruby_linker': attr.label(
      executable = True,
      default = Label('//tools/ruby:standalone_ruby'),
    )
  },
  outputs = {
    'manifest': '%{name}.tar_manifest',
  },
)
