load("//aos:config.bzl", "aos_config")

[
    aos_config(
        name = name,
        src = name + "_source.json",
        flatbuffers = [
            "//aos/network:remote_message_fbs",
            "//aos/events:ping_fbs",
            "//aos/network:message_bridge_client_fbs",
            "//aos/network:message_bridge_server_fbs",
            "//aos/network:timestamp_fbs",
        ],
        target_compatible_with = ["@platforms//os:linux"],
        visibility = ["//visibility:public"],
        deps = ["//aos/events:aos_config"],
    )
    for name in [
        "valid_multinode_config",
        "valid_singlenode_config",
        "multinode_no_statistics",
        "multinode_timestamp_typo",
        "multinode_extraneous_timestamp",
        "multinode_invalid_timestamp_logger_list",
        "multinode_common_logger",
        "multinode_no_logged_timestamps",
    ]
]
