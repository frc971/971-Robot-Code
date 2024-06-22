py_binary(
    name = "rotation",
    srcs = [
        "rotation.py",
    ],
    target_compatible_with = ["@platforms//cpu:x86_64"],
    deps = [
        "//frc971/control_loops/python:angular_system_current",
        "//frc971/control_loops/python:controls",
        "@pip//glog",
        "@pip//python_gflags",
    ],
)

genrule(
    name = "genrule_rotation",
    outs = [
        "rotation_plant.h",
        "rotation_plant.cc",
        "rotation_plant.json",
        "integral_rotation_plant.h",
        "integral_rotation_plant.cc",
        "integral_rotation_plant.json",
    ],
    cmd = "$(location :rotation) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [
        ":rotation",
    ],
)

filegroup(
    name = "rotation_json",
    srcs = ["integral_rotation_plant.json"],
    visibility = ["//visibility:public"],
)
