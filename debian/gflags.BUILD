py_library(
    name = "gflags",
    srcs = [
        "gflags.py",
        "gflags2man.py",
        "gflags_validators.py",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "gflags_googletest",
    srcs = [
        "tests/gflags_googletest.py",
    ],
)

py_test(
    name = "gflags_validators_test",
    size = "small",
    srcs = [
        "tests/gflags_validators_test.py",
    ],
    deps = [
        ":gflags",
        ":gflags_googletest",
    ],
)

py_library(
    name = "flags_modules_for_testing",
    srcs = [
        "tests/flags_modules_for_testing/__init__.py",
        "tests/flags_modules_for_testing/module_bar.py",
        "tests/flags_modules_for_testing/module_baz.py",
        "tests/flags_modules_for_testing/module_foo.py",
    ],
    deps = [
        ":gflags",
    ],
)

py_test(
    name = "gflags_unittest",
    size = "small",
    srcs = [
        "tests/gflags_unittest.py",
    ],
    deps = [
        ":flags_modules_for_testing",
        ":gflags",
        ":gflags_googletest",
    ],
)

py_test(
    name = "gflags_helpxml_test",
    size = "small",
    srcs = [
        "tests/gflags_helpxml_test.py",
    ],
    deps = [
        ":flags_modules_for_testing",
        ":gflags",
        ":gflags_googletest",
    ],
)
