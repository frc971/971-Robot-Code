load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")

def static_flatbuffer(name, src, visibility = None, deps = [], **kwargs):
    """Generates the code for the static C++ flatbuffer API for the specified fbs file.

    Generates a cc_library of name name that can be depended on by C++ code and other
    static_flatbuffer rules.

    The cc_library will consist of a single file suffixed with _static.h and prefixed
    with the name of the flatbuffer file itself (i.e., if you have a src of foo.fbs, then
    the resulting header will be foo_static.h).

    Args:
      name: Target name.
      src: .fbs file to generated code for.
      visibility: Desired rule visibility.
      deps: List of static_flatbuffer dependencies of this rule.
    """
    fbs_suffix = "_fbs"
    flatbuffer_cc_library(
        name = name + fbs_suffix,
        srcs = [src],
        deps = [dep + fbs_suffix for dep in deps],
        gen_reflections = True,
        visibility = visibility,
        **kwargs
    )

    # Until we make this a proper rule with providers or the such, we just manage headers
    # by having a strong convention where the header will be a function of the fbs name
    # rather than a function of the rule name.
    header_name = src.removesuffix(".fbs") + "_static.h"
    reflection_out = name + fbs_suffix + "_reflection_out"

    run_binary(
        name = name + "_gen",
        tool = "@org_frc971//aos/flatbuffers:generate_wrapper",
        srcs = [reflection_out],
        outs = [header_name],
        args = ["$(execpath %s)" % (reflection_out,), "$(execpath %s)" % (header_name,)],
    )
    native.cc_library(
        name = name,
        hdrs = [header_name],
        deps = ["@org_frc971//aos/flatbuffers:static_table", "@org_frc971//aos/flatbuffers:static_vector", name + fbs_suffix] + deps,
        visibility = visibility,
    )
    native.alias(
        name = name + "_reflection_out",
        actual = name + fbs_suffix + "_reflection_out",
    )
