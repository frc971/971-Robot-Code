# Additional constraint values not available in @platforms. Use these in things
# like select() statements or target_compatible_with.

package(default_visibility = ["//visibility:public"])

constraint_setting(name = "hardware")

constraint_value(
    name = "raspberry_pi",
    constraint_setting = ":hardware",
)

constraint_value(
    name = "roborio",
    constraint_setting = ":hardware",
)

constraint_value(
    name = "cortex_m4f",
    constraint_setting = ":hardware",
)

constraint_value(
    name = "cortex_m0plus",
    constraint_setting = ":hardware",
)
