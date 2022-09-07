FRC971's Python setup
================================================================================

How to depend on pip packages
--------------------------------------------------------------------------------
You can only depend on pip packages that are listed in our
[requirements file][requirements_file]. Any package you see in there can be
depended on.

For example, depend on `numpy` like so:
```python
py_binary(
    name = "bin",
    srcs = ["bin.py"],
    deps = [
        "@pip//numpy",
    ],
)
```

The labels are "normalized". That means the entries in the [requirements
file][requirements_file] may not be usable as-is. When you know the name of the
package, apply the following transformations:

1. Make the name lower-case.
2. Replace all dots and dashes with underscores.

The following are examples to show-case the various rules:

* `Jinja2` becomes `@pip//jinja2`.
* `absl-py` becomes `@pip//absl_py`.
* `Flask-SQLAlchemy` becomes `@pip//flask_sqlalchemy`.
* `ruamel.yaml` becomes `@pip//ruamel_yaml`.


How to add new pip packages
--------------------------------------------------------------------------------

1. Add the new package you're interested in to `tools/python/requirements.txt`.
2. Run the lock file generation script.

        bazel run //tools/python:requirements.update


How to make buildkite happy with new pip packages
--------------------------------------------------------------------------------
In order for buildkite to be able to use new pip packages, they have to be
mirrored on frc971 infrastructure.

1. Follow the above procedure for adding new pip packages if not already done.
2. Run the mirroring script.

        bazel run //tools/python:mirror_pip_packages --config=k8_upstream_python -- --ssh_host <software>

    where `<software>` is the `ssh(1)` target for reaching the server that hosts
    the FRC971 mirror.
