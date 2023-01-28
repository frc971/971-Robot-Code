def cc_static_flatbuffer(name, target, function, bfbs_name = None, visibility = None):
    """Creates a cc_library which encodes a file as a Span.

    args:
      target, The file to encode.
      function, The inline function, with full namespaces, to create.
      bfbs_name, For flatbuffer targets that have multiple fbs files, this
         specifies the basename of the bfbs file to generate a schema for.
    """
    native.genrule(
        name = name + "_gen",
        tools = ["@org_frc971//aos:flatbuffers_static"],
        srcs = [target],
        outs = [name + ".h"],
        cmd = "$(location @org_frc971//aos:flatbuffers_static) '$(SRCS)' $(OUTS) '" + function + "' " + (bfbs_name if bfbs_name else "-"),
    )

    native.cc_library(
        name = name,
        hdrs = [name + ".h"],
        deps = [
            "@com_google_absl//absl/types:span",
        ],
        visibility = visibility,
    )
