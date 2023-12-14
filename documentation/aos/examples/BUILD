load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "sensor_data_fbs",
    srcs = ["sensor_data.fbs"],
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
