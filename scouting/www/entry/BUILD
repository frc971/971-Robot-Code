load("@npm//:defs.bzl", "npm_link_all_packages")
load("//tools/build_rules:js.bzl", "ng_pkg")
load("//tools/build_rules:template.bzl", "jinja2_template")

npm_link_all_packages(name = "node_modules")

ng_pkg(
    name = "entry",
    extra_srcs = [
        ":action_helper.ts",
        "//scouting/www:app_common_css",
    ],
    deps = [
        ":node_modules",
        "//:node_modules/flatbuffers",
    ],
)

jinja2_template(
    name = "action_helper.ts",
    src = "action_helper.jinja2.ts",
    list_parameters = {
        # Is there a way to auto-generate the list of actions here? Would be
        # nice not to have a duplicate list here when they're already known in
        # the .fbs file.
        "ACTIONS": [
            "NoShowAction",
            "EndMatchAction",
            "MobilityAction",
            "PenaltyAction",
            "PickupCoralAction",
            "PlaceCoralAction",
            "ScoreCoralAction",
            "PickupAlgaeAction",
            "PlaceAlgaeAction",
            "ScoreAlgaeAction",
            "RobotDeathAction",
            "StartMatchAction",
            "DefenseAction",
            "EndAutoPhaseAction",
            "EndTeleopPhaseAction",
        ],
    },
)
