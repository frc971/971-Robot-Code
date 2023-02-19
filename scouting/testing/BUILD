py_binary(
    name = "scouting_test_servers",
    testonly = True,
    srcs = [
        "scouting_test_servers.py",
    ],
    data = [
        "//scouting",
        "//scouting/db/testdb_server",
        "//scouting/scraping:test_data",
        "@rules_python//python/runfiles",
    ],
    visibility = ["//visibility:public"],
)
