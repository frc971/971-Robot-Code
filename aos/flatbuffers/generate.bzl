load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:clean_dep.bzl", "aos_repo_name", "clean_dep")

def static_flatbuffer(name, visibility = None, deps = [], srcs = [], **kwargs):
    """Generates the code for the static C++ flatbuffer API for the specified fbs file.

    Generates a cc_library of name name that can be depended on by C++ code and other
    static_flatbuffer rules.

    The cc_library will consist of a single file suffixed with _static.h and prefixed
    with the name of the flatbuffer file itself (i.e., if you have a src of foo.fbs, then
    the resulting header will be foo_static.h).

    Args:
      name: Target name.
      srcs: List of fbs files to codegen.
      visibility: Desired rule visibility.
      deps: List of static_flatbuffer dependencies of this rule.
    """
    fbs_suffix = "_fbs"

    flatbuffer_cc_library(
        name = name + fbs_suffix,
        srcs = srcs,
        deps = [dep + fbs_suffix for dep in deps],
        gen_reflections = True,
        visibility = visibility,
        **kwargs
    )

    # Until we make this a proper rule with providers or the such, we just manage headers
    # by having a strong convention where the header will be a function of the fbs name
    # rather than a function of the rule name.
    header_names = [file.removesuffix(".fbs") + "_static.h" for file in srcs]
    reflection_out = name + fbs_suffix + "_reflection_out"

    run_binary(
        name = name + "_gen",
        tool = clean_dep("//aos/flatbuffers:generate_wrapper"),
        srcs = [reflection_out],
        outs = header_names,
        env = {
            "AOS_REPO_NAME": aos_repo_name(),
            "BFBS_FILES": "$(execpaths %s)" % (reflection_out,),
            "BASE_FILES": " ".join(srcs),
            "OUT_FILES": " ".join(["$(execpath %s)" % (name,) for name in header_names]),
        },
    )
    native.cc_library(
        name = name,
        hdrs = header_names,
        deps = [clean_dep("//aos/flatbuffers:static_table"), clean_dep("//aos/flatbuffers:static_vector"), name + fbs_suffix] + deps,
        visibility = visibility,
    )
    native.alias(
        name = name + "_reflection_out",
        actual = name + fbs_suffix + "_reflection_out",
        visibility = visibility,
    )
