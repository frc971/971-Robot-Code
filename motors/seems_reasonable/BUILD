py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:drivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    legacy_create_init = False,
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        ":python_init",
        "//frc971/control_loops/python:polydrivetrain",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

genrule(
    name = "genrule_drivetrain",
    outs = [
        "drivetrain_dog_motor_plant.h",
        "drivetrain_dog_motor_plant.cc",
        "kalman_drivetrain_motor_plant.h",
        "kalman_drivetrain_motor_plant.cc",
    ],
    cmd = "$(location :drivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:none"],
    tools = [
        ":drivetrain",
    ],
)

genrule(
    name = "genrule_polydrivetrain",
    outs = [
        "polydrivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.cc",
        "polydrivetrain_cim_plant.h",
        "polydrivetrain_cim_plant.cc",
        "hybrid_velocity_drivetrain.h",
        "hybrid_velocity_drivetrain.cc",
    ],
    cmd = "$(location :polydrivetrain) $(OUTS)",
    target_compatible_with = ["@platforms//os:none"],
    tools = [
        ":polydrivetrain",
    ],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain_dog_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.cc",
        # Note: skip the hybrid filter since we don't need it down here.
    ],
    hdrs = [
        "drivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.h",
    ],
    target_compatible_with = ["@platforms//os:none"],
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop",
        "//frc971/control_loops/drivetrain:polydrivetrain",
    ],
)

cc_library(
    name = "spring",
    srcs = ["spring.cc"],
    hdrs = ["spring.h"],
    visibility = ["//visibility:public"],
    deps = ["//frc971/zeroing:wrap"],
)

cc_test(
    name = "spring_test",
    srcs = [
        "spring_test.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        ":spring",
        "//aos/testing:googletest",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["//motors:python_init"],
)
