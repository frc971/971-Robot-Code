def _aos_downloader_impl(ctx):
  ctx.file_action(
    output = ctx.outputs.executable,
    executable = True,
    content = '\n'.join([
      '#!/bin/bash',
      'exec %s %s -- %s "$@"' % (ctx.executable._downloader.short_path,
                                 ' '.join([src.short_path for src in ctx.files.srcs]),
                                 ctx.attr.default_target),
    ]),
  )

  return struct(
    runfiles = ctx.runfiles(
      files = ctx.files.srcs + ctx.files._downloader,
      collect_data = True,
      collect_default = True,
    ),
    files = set([ctx.outputs.executable]),
  )

'''Creates a binary which downloads code to a robot.

Running this with `bazel run` will actually download everything.

Attrs:
  srcs: The files to download. They currently all get shoved into one folder.
  default_target: The default host to download to. If not specified, defaults to
                  roboRIO-971.local.
'''
aos_downloader = rule(
  implementation = _aos_downloader_impl,
  attrs = {
    '_downloader': attr.label(
      executable = True,
      cfg = HOST_CFG,
      default = Label('//aos/prime/downloader'),
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
)
