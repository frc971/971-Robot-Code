load("//tools/build_rules:ruby.bzl", "ruby_library", "ruby_binary")

ruby_library(
    name = "lib",
    srcs = [
        "load.rb",
        "write_iff_changed.rb",
    ] + glob(["*/*.rb"]),
)

ruby_binary(
    name = "compiler",
    srcs = [
        "compiler.rb",
    ],
    data = [
        "@clang_3p6_repo//:clang-format",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":lib",
    ],
)

ruby_binary(
    name = "queue_primitives",
    srcs = [
        "queue_primitives.rb",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":lib",
    ],
)

ruby_binary(
    name = "print_field",
    srcs = [
        "print_field.rb",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":lib",
    ],
)
