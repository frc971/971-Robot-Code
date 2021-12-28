# TODO(phil): Delete this file after migrating everything to use upstream pip
# rules.

package(default_visibility = ["//visibility:public"])

constraint_setting(
    name = "python_source",
    default_constraint_value = ":debian_bundled_python",
)

# Python code taken from .deb packages.
constraint_value(
    name = "debian_bundled_python",
    constraint_setting = ":python_source",
)

# Python code taken from pypi.org.
constraint_value(
    name = "upstream_bundled_python",
    constraint_setting = ":python_source",
)
