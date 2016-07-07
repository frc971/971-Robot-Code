def _aos_downloader_impl(ctx):
  all_files = ctx.files.srcs + ctx.files.start_srcs + [ctx.outputs._startlist]
  ctx.file_action(
    output = ctx.outputs.executable,
    executable = True,
    content = '\n'.join([
      '#!/bin/bash',
      'set -e',
      'cd "${BASH_SOURCE[@]}.runfiles/%s"' % ctx.workspace_name,
    ] + ['%s %s --dirs %s -- %s "$@"' % (
       ctx.executable._downloader.short_path,
       ' '.join([src.short_path for src in d.downloader_srcs]),
       d.downloader_dir,
       ctx.attr.default_target) for d in ctx.attr.dirs] + [
      'exec %s %s -- %s "$@"' % (ctx.executable._downloader.short_path,
                                 ' '.join([src.short_path for src in all_files]),
                                 ctx.attr.default_target),
    ]),
  )

  ctx.file_action(
    output = ctx.outputs._startlist,
    content = '\n'.join([f.basename for f in ctx.files.start_srcs]) + '\n',
  )

  to_download = [ctx.outputs._startlist]
  to_download += all_files
  for d in ctx.attr.dirs:
    to_download += d.downloader_srcs

  return struct(
    runfiles = ctx.runfiles(
      files = to_download + ctx.files._downloader,
      collect_data = True,
      collect_default = True,
    ),
    files = set([ctx.outputs.executable]),
  )

def _aos_downloader_dir_impl(ctx):
  return struct(
    downloader_dir = ctx.attr.dir,
    downloader_srcs = ctx.files.srcs
  )

'''Creates a binary which downloads code to a robot.

Running this with `bazel run` will actually download everything.

This also generates a start_list.txt file with the names of binaries to start.

Attrs:
  srcs: The files to download. They currently all get shoved into one folder.
  dirs: A list of aos_downloader_dirs to download too.
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
    'dirs': attr.label_list(
      mandatory = False,
      providers = [
        'downloader_dir',
        'downloader_srcs',
      ]
    ),
    'default_target': attr.string(
      default = 'roboRIO-971-frc.local',
    ),
  },
  executable = True,
  outputs = {
    '_startlist': '%{name}.start_list.dir/start_list.txt',
  },
)

'''Downloads files to a specific directory.

This rule does nothing by itself. Use it by adding to the dirs attribute of an
aos_downloader rule.

Attrs:
  srcs: The files to download. They all go in the same directory.
  dir: The directory (relative to the standard download directory) to put all
       the files in.
'''
aos_downloader_dir = rule(
  implementation = _aos_downloader_dir_impl,
  attrs = {
    'srcs': attr.label_list(
      mandatory = True,
      allow_files = True,
    ),
    'dir': attr.string(
       mandatory = True,
    ),
  },
)
