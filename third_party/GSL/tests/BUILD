licenses(["notice"])

_copts = [
    "-fno-strict-aliasing",
    "-Wall",
    "-Wcast-align",
    "-Wconversion",
    "-Wctor-dtor-privacy",
    "-Werror",
    "-Wextra",
    "-Wno-missing-braces",
    "-Wnon-virtual-dtor",
    "-Wold-style-cast",
    "-Woverloaded-virtual",
    "-Wpedantic",
    "-Wshadow",
    "-Wsign-conversion",
]

[cc_test(
    name = filename[:-5],
    srcs = [filename],
    copts = _copts + [
        "-DNO_SELFTEST=true",
        "-fexceptions",
        "-frtti",
        "-DGSL_THROW_ON_CONTRACT_VIOLATION",
    ],
    deps = [
        ":test",
        "//third_party/Catch2",
        "//third_party/GSL",
    ],
) for filename in glob(
    include = ["*_tests.cpp"],
    exclude = ["no_exception*"],
)]

[cc_test(
    name = filename[:-5],
    srcs = [filename],
    copts = _copts + [
        "-DGSL_TERMINATE_ON_CONTRACT_VIOLATION",
    ],
    deps = [
        ":test",
        "//third_party/Catch2",
        "//third_party/GSL",
    ],
) for filename in glob(["no_exception*_tests.cpp"])]

cc_library(
    name = "test",
    srcs = ["test.cpp"],
    copts = _copts + [
        "-DNO_SELFTEST=true",
        "-fexceptions",
        "-frtti",
        "-DGSL_THROW_ON_CONTRACT_VIOLATION",
    ],
    deps = [
        "//third_party/Catch2",
        "//third_party/GSL",
    ],
)
