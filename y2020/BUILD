load("//frc971:downloader.bzl", "robot_downloader")
load("//aos:config.bzl", "aos_config")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:template.bzl", "jinja2_template")

robot_downloader(
    binaries = [
        ":setpoint_setter",
        "//aos/network:web_proxy_main",
    ],
    data = [
        ":config",
    ],
    dirs = [
        "//y2020/actors:splines",
        "//y2020/www:www_files",
    ],
    start_binaries = [
        "//aos/events/logging:logger_main",
        "//aos/network:web_proxy_main",
        ":joystick_reader",
        ":wpilib_interface",
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//y2020/actors:binaries",
        "//y2020/control_loops/drivetrain:drivetrain",
        "//y2020/control_loops/drivetrain:trajectory_generator",
        "//y2020/control_loops/superstructure:superstructure",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

robot_downloader(
    name = "pi_download",
    binaries = [
        "//y2020/vision:viewer",
    ],
    data = [
        ":config",
    ],
    dirs = [
        "//y2020/www:www_files",
    ],
    start_binaries = [
        "//aos/network:message_bridge_client",
        "//aos/network:message_bridge_server",
        "//aos/network:web_proxy_main",
        "//y2020/vision:camera_reader",
    ],
    target_compatible_with = ["//tools/platforms/hardware:raspberry_pi"],
    target_type = "pi",
)

cc_library(
    name = "constants",
    srcs = [
        "constants.cc",
    ],
    hdrs = [
        "constants.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/logging",
        "//aos/network:team_number",
        "//aos/stl_mutex",
        "//frc971:constants",
        "//frc971/control_loops:static_zeroing_single_dof_profiled_subsystem",
        "//frc971/shooter_interpolation:interpolation",
        "//y2020/control_loops/drivetrain:polydrivetrain_plants",
        "//y2020/control_loops/superstructure/accelerator:accelerator_plants",
        "//y2020/control_loops/superstructure/control_panel:control_panel_plants",
        "//y2020/control_loops/superstructure/finisher:finisher_plants",
        "//y2020/control_loops/superstructure/hood:hood_plants",
        "//y2020/control_loops/superstructure/intake:intake_plants",
        "//y2020/control_loops/superstructure/turret:turret_plants",
        "@com_google_absl//absl/base",
    ],
)

cc_binary(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cc",
    ],
    target_compatible_with = ["//tools/platforms/hardware:roborio"],
    deps = [
        ":constants",
        "//aos:init",
        "//aos:math",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/stl_mutex",
        "//aos/time",
        "//aos/util:log_interval",
        "//aos/util:phased_loop",
        "//aos/util:wrapping_counter",
        "//frc971/autonomous:auto_mode_fbs",
        "//frc971/control_loops:control_loop",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_position_fbs",
        "//frc971/input:robot_state_fbs",
        "//frc971/wpilib:ADIS16470",
        "//frc971/wpilib:buffered_pcm",
        "//frc971/wpilib:drivetrain_writer",
        "//frc971/wpilib:encoder_and_potentiometer",
        "//frc971/wpilib:interrupt_edge_counting",
        "//frc971/wpilib:joystick_sender",
        "//frc971/wpilib:logging_fbs",
        "//frc971/wpilib:loop_output_handler",
        "//frc971/wpilib:pdp_fetcher",
        "//frc971/wpilib:sensor_reader",
        "//frc971/wpilib:wpilib_interface",
        "//frc971/wpilib:wpilib_robot_base",
        "//third_party:phoenix",
        "//third_party:wpilib",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
    ],
)

cc_binary(
    name = "joystick_reader",
    srcs = [
        ":joystick_reader.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":setpoint_fbs",
        "//aos:init",
        "//aos/actions:action_lib",
        "//aos/logging",
        "//frc971/autonomous:auto_fbs",
        "//frc971/autonomous:base_autonomous_actor",
        "//frc971/control_loops:profiled_subsystem_fbs",
        "//frc971/input:action_joystick_input",
        "//frc971/input:drivetrain_input",
        "//frc971/input:joystick_input",
        "//y2020:constants",
        "//y2020/control_loops/drivetrain:drivetrain_base",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
    ],
)

aos_config(
    name = "config",
    src = "y2020.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision:vision_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":config_laptop",
        ":config_pi1",
        ":config_pi2",
        ":config_pi3",
        ":config_pi4",
        ":config_pi5",
        ":config_roborio",
    ],
)

[
    aos_config(
        name = "config_" + pi,
        src = "y2020_" + pi + ".json",
        flatbuffers = [
            "//aos/network:message_bridge_client_fbs",
            "//aos/network:message_bridge_server_fbs",
            "//aos/network:timestamp_fbs",
            "//y2020/vision/sift:sift_fbs",
            "//y2020/vision/sift:sift_training_fbs",
            "//y2020/vision:vision_fbs",
            "//aos/network:remote_message_fbs",
            "//y2020/vision:galactic_search_path_fbs",
        ],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        deps = [
            "//aos/events:config",
            "//frc971/control_loops/drivetrain:config",
            "//frc971/input:config",
        ],
    )
    for pi in [
        "pi1",
        "pi2",
        "pi3",
        "pi4",
        "pi5",
    ]
]

aos_config(
    name = "config_laptop",
    src = "y2020_laptop.json",
    flatbuffers = [
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//y2020/vision/sift:sift_fbs",
        "//y2020/vision/sift:sift_training_fbs",
        "//y2020/vision:vision_fbs",
        "//aos/network:remote_message_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/events:config",
        "//frc971/control_loops/drivetrain:config",
        "//frc971/input:config",
    ],
)

aos_config(
    name = "config_roborio",
    src = "y2020_roborio.json",
    flatbuffers = [
        ":setpoint_fbs",
        "//aos/network:remote_message_fbs",
        "//aos/network:message_bridge_client_fbs",
        "//aos/network:message_bridge_server_fbs",
        "//aos/network:timestamp_fbs",
        "//y2020/control_loops/superstructure:superstructure_goal_fbs",
        "//y2019/control_loops/drivetrain:target_selector_fbs",
        "//y2020/control_loops/drivetrain:localizer_debug_fbs",
        "//y2020/control_loops/superstructure:superstructure_output_fbs",
        "//y2020/control_loops/superstructure:superstructure_position_fbs",
        "//y2020/control_loops/superstructure:superstructure_status_fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/events:config",
        "//frc971/autonomous:config",
        "//frc971/control_loops/drivetrain:config",
        "//frc971/input:config",
        "//frc971/wpilib:config",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "log_web_proxy",
    srcs = ["log_web_proxy.sh"],
    data = [
        ":config",
        "//aos/network:log_web_proxy_main",
        "//y2020/www:camera_main_bundle.min.js",
        "//y2020/www:field_main_bundle.min.js",
        "//y2020/www:files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

sh_binary(
    name = "web_proxy",
    srcs = ["web_proxy.sh"],
    data = [
        ":config",
        "//aos/network:web_proxy_main",
        "//y2020/www:camera_main_bundle.min.js",
        "//y2020/www:field_main_bundle.min.js",
        "//y2020/www:files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "setpoint_fbs",
    srcs = [
        "setpoint.fbs",
    ],
    gen_reflections = 1,
    target_compatible_with = ["@platforms//os:linux"],
)

cc_binary(
    name = "setpoint_setter",
    srcs = ["setpoint_setter.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":setpoint_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
    ],
)

[
    jinja2_template(
        name = "y2020_pi" + str(num) + ".json",
        src = "y2020_pi_template.json",
        parameters = {"NUM": str(num)},
        target_compatible_with = ["@platforms//os:linux"],
    )
    for num in range(1, 6)
]
