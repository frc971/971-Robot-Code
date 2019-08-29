genrule(
    name = "genrule_column",
    outs = [
        "column_plant.h",
        "column_plant.cc",
        "column_integral_plant.h",
        "column_integral_plant.cc",
        "stuck_column_integral_plant.h",
        "stuck_column_integral_plant.cc",
    ],
    cmd = "$(location //y2017/control_loops/python:column) $(OUTS)",
    tools = [
        "//y2017/control_loops/python:column",
    ],
)

cc_library(
    name = "column_plants",
    srcs = [
        "column_integral_plant.cc",
        "column_plant.cc",
        "stuck_column_integral_plant.cc",
    ],
    hdrs = [
        "column_integral_plant.h",
        "column_plant.h",
        "stuck_column_integral_plant.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
    ],
)

cc_library(
    name = "column",
    srcs = [
        "column.cc",
    ],
    hdrs = [
        "column.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":column_plants",
        ":column_zeroing",
        "//aos:math",
        "//aos/controls:control_loop",
        "//frc971/control_loops:profiled_subsystem",
        "//y2017:constants",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
        "//y2017/control_loops/superstructure:vision_time_adjuster",
        "//y2017/control_loops/superstructure/intake",
    ],
)

cc_library(
    name = "column_zeroing",
    srcs = [
        "column_zeroing.cc",
    ],
    hdrs = [
        "column_zeroing.h",
    ],
    deps = [
        "//frc971:constants",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/zeroing",
        "//frc971/zeroing:wrap",
        "//y2017:constants",
        "//y2017/control_loops/superstructure:superstructure_position_fbs",
        "//y2017/control_loops/superstructure:superstructure_status_fbs",
    ],
)

cc_test(
    name = "column_zeroing_test",
    srcs = [
        "column_zeroing_test.cc",
    ],
    deps = [
        ":column_zeroing",
        "//aos/testing:test_shm",
        "//frc971/control_loops:position_sensor_sim",
        "//frc971/control_loops:team_number_test_environment",
        "//y2017:constants",
    ],
)
