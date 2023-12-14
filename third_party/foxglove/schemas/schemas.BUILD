load("@com_github_google_flatbuffers//:build_defs.bzl", "DEFAULT_FLATC_ARGS", "flatbuffer_cc_library")
load("@org_frc971//aos/flatbuffers:generate.bzl", "static_flatbuffer")

FLATC_ARGS = [arg for arg in DEFAULT_FLATC_ARGS if arg != "--require-explicit-ids"]

static_flatbuffer(
    name = "schemas",
    srcs = glob(["*.fbs"]),
    flatc_args = FLATC_ARGS,
    visibility = ["//visibility:public"],
)

load("@org_frc971//aos:flatbuffers.bzl", "cc_static_flatbuffer")

[cc_static_flatbuffer(
    name = filename[:-4] + "_schema",
    bfbs_name = filename[:-4] + ".bfbs",
    function = "foxglove::" + filename[:-4] + "Schema",
    target = ":schemas_reflection_out",
    visibility = ["//visibility:public"],
) for filename in glob(["*.fbs"])]
