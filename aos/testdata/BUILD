load("//aos:config.bzl", "aos_config")

filegroup(
    name = "test_configs",
    srcs = [
        "backwards.json",
        "config1.json",
        "config1_bad.json",
        "config1_multinode.json",
        "config2.json",
        "config2_multinode.json",
        "config3.json",
        "expected.json",
        "expected_merge_with.json",
        "expected_multinode.json",
        "good_multinode.json",
        "good_multinode_hostnames.json",
        "invalid_channel_name1.json",
        "invalid_channel_name2.json",
        "invalid_channel_name3.json",
        "invalid_destination_node.json",
        "invalid_nodes.json",
        "invalid_source_node.json",
        "self_forward.json",
    ],
    visibility = ["//aos:__pkg__"],
)

genrule(
    name = "generated_test_config",
    outs = ["generated.json"],
    cmd = "echo '" +
          """{
  "imports": [
    "empty.json"
  ]
}""" + "'> $@",
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "empty_config",
    src = "empty.json",
    target_compatible_with = ["@platforms//os:linux"],
)

aos_config(
    name = "generated_config",
    src = "generated.json",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":empty_config"],
)

aos_config(
    name = "generated_config_flattener_test",
    src = "generated_config_test.json",
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":generated_config"],
)
