def _aos_vision_downloader_impl(ctx):
  all_files = ctx.files.srcs
  ctx.file_action(
    output = ctx.outputs.executable,
    executable = True,
    content = '\n'.join([
      '#!/bin/bash',
      'set -e',
      'cd "${BASH_SOURCE[0]}.runfiles/%s"' % ctx.workspace_name,
    ] + [
      'exec %s %s -- %s "$@"' % (ctx.executable._downloader.short_path,
                                 ' '.join([src.short_path for src in all_files]),
                                 ctx.attr.default_target),
    ]),
  )

  to_download = all_files

  return struct(
    runfiles = ctx.runfiles(
      files = to_download + ctx.files._downloader,
      collect_data = True,
      collect_default = True,
    ),
    files = set([ctx.outputs.executable]),
  )

'''Creates a binary which downloads code to a robot camera processing unit.

Running this with `bazel run` will actually download everything.

Attrs:
  srcs: The files to download. They currently all get shoved into one folder.
  default_target: The default host to download to. If not specified, defaults to
                  root@10.9.71.179.
'''
aos_vision_downloader = rule(
  implementation = _aos_vision_downloader_impl,
  attrs = {
    '_downloader': attr.label(
      executable = True,
      cfg = 'host',
      default = Label('//aos/vision/download:downloader'),
    ),
    'srcs': attr.label_list(
      mandatory = True,
      allow_files = True,
    ),
    'default_target': attr.string(
      default = 'root@10.9.71.179',
    ),
  },
  executable = True,
)
