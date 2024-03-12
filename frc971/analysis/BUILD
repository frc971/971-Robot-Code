load("//tools/build_rules:js.bzl", "rollup_bundle", "ts_project")

package(default_visibility = ["//visibility:public"])

ts_project(
    name = "plot_index",
    srcs = ["plot_index.ts"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:configuration_ts_fbs",
        "//aos/network/www:demo_plot",
        "//aos/network/www:proxy",
        "//frc971/control_loops/drivetrain:down_estimator_plotter",
        "//frc971/control_loops/drivetrain:drivetrain_plotter",
        "//frc971/control_loops/drivetrain:robot_state_plotter",
        "//frc971/control_loops/drivetrain:spline_plotter",
        "//frc971/wpilib:imu_plotter",
        "//y2020/control_loops/drivetrain:localizer_plotter",
        "//y2020/control_loops/superstructure:accelerator_plotter",
        "//y2020/control_loops/superstructure:finisher_plotter",
        "//y2020/control_loops/superstructure:hood_plotter",
        "//y2020/control_loops/superstructure:turret_plotter",
        "//y2021_bot3/control_loops/superstructure:superstructure_plotter",
        "//y2022/control_loops/superstructure:catapult_plotter",
        "//y2022/control_loops/superstructure:climber_plotter",
        "//y2022/control_loops/superstructure:intake_plotter",
        "//y2022/control_loops/superstructure:superstructure_plotter",
        "//y2022/control_loops/superstructure:turret_plotter",
        "//y2022/localizer:localizer_plotter",
        "//y2022/vision:vision_plotter",
        "//y2023/control_loops/superstructure:superstructure_plotter",
        "//y2023/localizer:corrections_plotter",
        "//y2023/localizer:localizer_plotter",
        "//y2024/control_loops/superstructure:superstructure_plotter",
        "//y2024/localizer:corrections_plotter",
        "//y2024/localizer:localizer_plotter",
    ],
)

rollup_bundle(
    name = "plot_index_bundle",
    entry_point = "plot_index.ts",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":plot_index",
    ],
)

filegroup(
    name = "plotter_files",
    srcs = [
        "index.html",
        "plot_index_bundle.min.js",
        "styles.css",
    ],
)

sh_binary(
    name = "web_plotter",
    srcs = ["web_plotter.sh"],
    data = [
        ":plotter_files",
        "//aos/network:log_web_proxy_main",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

sh_binary(
    name = "live_web_plotter_demo",
    srcs = ["live_web_plotter_demo.sh"],
    data = [
        ":plotter_files",
        "//aos/network:web_proxy_main",
        "//aos/network/www:test_config",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

genrule(
    name = "copy_css",
    srcs = [
        "//aos/network/www:styles.css",
    ],
    outs = ["styles.css"],
    cmd = "cp $< $@",
)

cc_binary(
    name = "trim_log_to_enabled",
    srcs = [
        "trim_log_to_enabled.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:simulation_logger",
        "//frc971/input:joystick_state_fbs",
    ],
)

cc_binary(
    name = "log_to_match",
    srcs = [
        "log_to_match.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//frc971/input:joystick_state_fbs",
    ],
)

py_binary(
    name = "trim_and_plot_foxglove",
    srcs = ["trim_and_plot_foxglove.py"],
    data = [
        ":trim_log_to_enabled",
        "//aos/util:log_to_mcap",
        "@foxglove_studio",
    ],
    deps = ["@RangeHTTPServer"],
)

cc_binary(
    name = "pdp_values",
    srcs = [
        "pdp_values.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:init",
        "//aos/events:simulated_event_loop",
        "//aos/events/logging:log_reader",
        "//aos/util:simulation_logger",
        "//frc971/wpilib:pdp_values_fbs",
    ],
)
