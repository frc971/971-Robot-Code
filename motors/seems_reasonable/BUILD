load("//tools:environments.bzl", "mcu_cpus")

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:drivetrain",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    restricted_to = ["//tools:k8"],
    deps = [
        "//external:python-gflags",
        "//external:python-glog",
        "//frc971/control_loops/python:polydrivetrain",
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
    restricted_to = mcu_cpus,
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
    ],
    cmd = "$(location :polydrivetrain) $(OUTS)",
    restricted_to = mcu_cpus,
    tools = [
        ":polydrivetrain",
    ],
)

cc_library(
    name = "drivetrain_lib",
    srcs = [
        "drivetrain_dog_motor_plant.cc",
        "polydrivetrain_dog_motor_plant.cc",
    ],
    hdrs = [
        "drivetrain_dog_motor_plant.h",
        "polydrivetrain_dog_motor_plant.h",
    ],
    restricted_to = mcu_cpus,
    visibility = ["//visibility:public"],
    deps = [
        "//frc971/control_loops:state_feedback_loop_uc",
        "//frc971/control_loops/drivetrain:polydrivetrain_uc",
    ],
)
