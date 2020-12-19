def cc_static_flatbuffer(name, target, function):
    """Creates a cc_library which encodes a file as a Span.

    args:
      target, The file to encode.
      function, The inline function, with full namespaces, to create.
    """
    native.genrule(
        name = name + "_gen",
        tools = ["//aos:flatbuffers_static"],
        srcs = [target],
        outs = [name + ".h"],
        cmd = "$(location //aos:flatbuffers_static) $(SRCS) $(OUTS) '" + function + "'",
    )

    native.cc_library(
        name = name,
        hdrs = [name + ".h"],
        deps = [
            "@com_google_absl//absl/types:span",
        ],
    )
