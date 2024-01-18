load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")

def drivetrain_config(name, out):
    """Merges the JSONs for a drivetrain into a single output.

    This is meant to be used in the same folder as the genrule's for
    the drivetrain control loops for a given year, and will output
    a JSON file that can be included in a constants.json.
    """
    srcs = [":drivetrain_dog_motor_plant.json", ":kalman_drivetrain_motor_plant.json", ":hybrid_velocity_drivetrain.json", ":polydrivetrain_dog_motor_plant.json"]
    run_binary(
        name = name,
        tool = "//frc971/control_loops/drivetrain:drivetrain_config_merge",
        srcs = srcs,
        outs = [out],
        args = ["$(location %s)" % (file,) for file in srcs] + ["$(location %s)" % (out,)],
        visibility = ["//visibility:public"],
    )
