def _aos_downloader_impl(ctx):
  all_files = ctx.files.srcs + ctx.files.start_srcs + [ctx.outputs._startlist]
  ctx.file_action(
    output = ctx.outputs.executable,
    executable = True,
    content = '\n'.join([
      '#!/bin/bash',
      'cd "${BASH_SOURCE[@]}.runfiles"',
      'exec %s %s -- %s "$@"' % (ctx.executable._downloader.short_path,
                                 ' '.join([src.short_path for src in all_files]),
                                 ctx.attr.default_target),
    ]),
  )

  ctx.file_action(
    output = ctx.outputs._startlist,
    content = '\n'.join([f.basename for f in ctx.files.start_srcs]) + '\n',
  )

  return struct(
    runfiles = ctx.runfiles(
      files = all_files + ctx.files._downloader + [ctx.outputs._startlist],
      collect_data = True,
      collect_default = True,
    ),
    files = set([ctx.outputs.executable]),
  )

'''Creates a binary which downloads code to a robot.

Running this with `bazel run` will actually download everything.

This also generates a start_list.txt file with the names of binaries to start.

Attrs:
  srcs: The files to download. They currently all get shoved into one folder.
  start_srcs: Like srcs, except they also get put into start_list.txt.
  default_target: The default host to download to. If not specified, defaults to
                  roboRIO-971.local.
'''
aos_downloader = rule(
  implementation = _aos_downloader_impl,
  attrs = {
    '_downloader': attr.label(
      executable = True,
      cfg = HOST_CFG,
      default = Label('//aos/downloader'),
    ),
    'start_srcs': attr.label_list(
      mandatory = True,
      allow_files = True,
    ),
    'srcs': attr.label_list(
      mandatory = True,
      allow_files = True,
    ),
    'default_target': attr.string(
      default = 'roboRIO-971.local',
    ),
  },
  executable = True,
  outputs = {
    '_startlist': '%{name}.start_list.dir/start_list.txt',
  },
)
