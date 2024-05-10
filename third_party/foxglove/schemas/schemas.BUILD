load("@com_github_google_flatbuffers//:build_defs.bzl", "DEFAULT_FLATC_ARGS", "flatbuffer_cc_library")
load("@org_frc971//aos/flatbuffers:generate.bzl", "static_flatbuffer")

FLATC_ARGS = [arg for arg in DEFAULT_FLATC_ARGS if arg != "--require-explicit-ids"]

NON_TABLE_SCHEMAS = [
    "Duration.fbs",
    "Time.fbs",
]

static_flatbuffer(
    name = "non_table_schemas",
    srcs = NON_TABLE_SCHEMAS,
    flatc_args = FLATC_ARGS,
    visibility = ["//visibility:public"],
)

static_flatbuffer(
    name = "schemas",
    srcs = glob(
        ["*.fbs"],
        exclude = NON_TABLE_SCHEMAS,
    ),
    flatc_args = FLATC_ARGS,
    visibility = ["//visibility:public"],
    deps = [":non_table_schemas"],
)

load("@org_frc971//aos:flatbuffers.bzl", "cc_static_flatbuffer")

[cc_static_flatbuffer(
    name = filename[:-4] + "_schema",
    bfbs_name = filename[:-4] + ".bfbs",
    function = "foxglove::" + filename[:-4] + "Schema",
    target = ":schemas_reflection_out",
    visibility = ["//visibility:public"],
) for filename in glob(["*.fbs"])]
