def _do_proto_cc_library_impl(ctx):
  deps = [ctx.file.src]

  for dep in ctx.attr.deps:
    deps += dep.proto.deps

  message = 'Building %s and %s from %s' % (ctx.outputs.pb_h.short_path,
                                            ctx.outputs.pb_cc.short_path,
                                            ctx.file.src.short_path)
  ctx.action(
    inputs = deps + ctx.files._well_known_protos,
    executable = ctx.executable._protoc,
    arguments = [
      '--cpp_out=%s' % ctx.configuration.genfiles_dir.path,
      '-I.',
      '-Ithird_party/protobuf/src',
      ctx.file.src.path,
    ],
    mnemonic = 'ProtocCc',
    progress_message = message,
    outputs = [
      ctx.outputs.pb_h,
      ctx.outputs.pb_cc,
    ],
  )

  return struct(
    proto = struct(
      deps = deps,
    )
  )

def _do_proto_cc_library_outputs(src):
  basename = src.name[:-len('.proto')]
  return {
    'pb_h': '%s.pb.h' % basename,
    'pb_cc': '%s.pb.cc' % basename,
  }

_do_proto_cc_library = rule(
  implementation = _do_proto_cc_library_impl,
  attrs = {
    'src': attr.label(
      allow_files = FileType(['.proto']),
      mandatory = True,
      single_file = True,
    ),
    'deps': attr.label_list(providers = ["proto"]),
    '_protoc': attr.label(
      default = Label('//third_party/protobuf:protoc'),
      executable = True,
      cfg = 'host',
    ),
    '_well_known_protos': attr.label(
      default = Label('//third_party/protobuf:well_known_protos'),
    ),
  },
  outputs = _do_proto_cc_library_outputs,
  output_to_genfiles = True,
)

def proto_cc_library(name, src, deps = [], visibility = None):
  '''Generates a cc_library from a single .proto file. Does not support
  dependencies on any .proto files except the well-known ones protobuf comes
  with (which are unconditionally depended on).

  Attrs:
    src: The .proto file.
  '''

  _do_proto_cc_library(
    name = '%s__proto_srcs' % name,
    src = src,
    deps = [('%s__proto_srcs' % o_name) for o_name in deps],
    visibility = visibility,
  )
  basename = src[:-len('.proto')]
  native.cc_library(
    name = name,
    srcs = [ '%s.pb.cc' % basename ],
    hdrs = [ '%s.pb.h' % basename ],
    deps = [ '//third_party/protobuf' ] + deps,
    visibility = visibility,
  )
