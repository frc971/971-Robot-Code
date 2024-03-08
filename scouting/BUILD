load("//tools/build_rules:apache.bzl", "apache_wrapper")
load("//tools/build_rules:js.bzl", "cypress_test")

sh_binary(
    name = "scouting",
    srcs = [
        "scouting.sh",
    ],
    data = [
        "//scouting/webserver",
        "//scouting/www:static_files",
    ],
    tags = [
        "no-remote-cache",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)

# TODO(philipp): Sandbox the following:
# - libnss3
# - libdrm2
# - libgbm1
# - x11-xkb-utils (See TODO in scouting/scouting_test_runner.js)
cypress_test(
    name = "scouting_test",
    data = [
        "scouting_test.cy.js",
        "test_img_1.png",
        "test_img_2.png",
        "//scouting/testing:scouting_test_servers",
    ],
    runner = "scouting_test_runner.js",
    tags = [
        "no-remote-cache",
    ],
)

# The QR code test is separate from scouting_test because it's slow. Most of
# the time folks will want to iterate on `scouting_test`.
cypress_test(
    name = "scouting_qrcode_test",
    size = "large",
    data = [
        "scouting_qrcode_test.cy.js",
        "//scouting/testing:scouting_test_servers",
        "//scouting/testing/camera_simulator",
    ],
    runner = "scouting_test_runner.js",
    tags = [
        "no-remote-cache",
    ],
)

apache_wrapper(
    name = "https",
    binary = ":scouting",
)
