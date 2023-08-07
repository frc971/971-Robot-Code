load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "sensor_data_fbs",
    srcs = ["sensor_data.fbs"],
    gen_reflections = True,
)

cc_library(
    name = "clock_offset_reader",
    srcs = ["clock_offset_reader.cc"],
    hdrs = ["clock_offset_reader.h"],
    deps = [
        ":sensor_data_fbs",
        "//aos/events:event_loop",
        "//aos/network:message_bridge_server_fbs",
    ],
)
