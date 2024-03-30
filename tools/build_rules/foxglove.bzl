load("@aspect_rules_js//js:defs.bzl", "js_run_binary")

def foxglove_extension(name, **kwargs):
    """Compiles a foxglove extension into a .foxe file.

    Drag the generated .foxe file onto the foxglove UI in your browser. The
    extension should then install automatically. If you want to update the
    extension, drag a new version to the UI.

    Use `tools/foxglove/create-foxglove-extension`. Don't use this rule
    directly. See `tools/foxglove/README.md` for more details.

    Args:
        name: The name of the target.
        **kwargs: The arguments to pass to js_run_binary.
    """

    # We need to glob all the non-Bazel files because we're going to invoke the
    # `foxglove-extension` binary directly. That expects to have access to
    # `package.json` and the like.
    all_files = native.glob(
        ["**"],
        exclude = [
            "BUILD",
            "BUILD.bazel",
        ],
    )

    # Run the `foxglove-extension` wrapper to create the .foxe file.
    js_run_binary(
        name = name,
        srcs = all_files + [
            ":node_modules",
        ],
        tool = "//tools/foxglove:foxglove_extension_wrapper",
        outs = ["%s.foxe" % name],
        args = [
            "package",
            "--out",
            "%s.foxe" % name,
        ],
        target_compatible_with = [
            "@platforms//cpu:x86_64",
        ],
        **kwargs
    )
