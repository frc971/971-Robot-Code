def _gen_embedded_impl(ctx):
  ctx.action(
    inputs = ctx.files.srcs,
    outputs = [ ctx.outputs.header ],
    executable = ctx.executable._gen_embedded,
    arguments = [ ctx.outputs.header.path ] + [f.path for f in ctx.files.srcs],
    progress_message = 'Generating %s' % ctx.outputs.header.short_path,
    mnemonic = 'GenEmbedded',
  )

  return struct(
    files = set([ ctx.outputs.header ]),
  )

_do_gen_embedded = rule(
  implementation = _gen_embedded_impl,
  attrs = {
    'srcs': attr.label_list(
      mandatory = True,
      non_empty = True,
      allow_files = True,
    ),
    '_gen_embedded': attr.label(
      executable = True,
      default = Label('//aos/externals/seasocks:gen_embedded'),
      cfg = 'host',
    ),
  },
  outputs = {
    'header': 'embedded.h',
  },
  output_to_genfiles = True,
)

'''Generates the header for Seasocks to load the embedded files.

This always outputs a file named "embedded.h" in the current package, so there
can be a maximum of one of these rules in each package.

Attrs:
  srcs: Files to allow loading.
'''
def gen_embedded(name, srcs, visibility = None):
  _do_gen_embedded(
    name = name + '__do_gen',
    visibility = ['//visibility:private'],
    srcs = srcs,
  )
  native.cc_library(
    name = name,
    visibiltity = visibility,
    hdrs = [
      ':%s__do_gen' % name,
    ],
  )
