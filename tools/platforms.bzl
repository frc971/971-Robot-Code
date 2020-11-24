load("@bazel_skylib//lib:selects.bzl", "selects")

def _any_of(constraint_values):
    return selects.with_or({
        tuple(constraint_values): [],
        "//conditions:default": ["@platforms//:incompatible"],
    })

platforms = struct(
    any_of = _any_of,
)
