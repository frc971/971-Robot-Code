# This file contains replacements for select where the keys have more abstract
# meanings so we can map multiple conditions to the same value easily and
# quickly find issues where something new isn't handled.
# It will also make adding ORs when it makes sense easy to do nicely.

# TODO(james): Decide what to do about webassembly/emscripten CPU and
# compiler configurations. Bazel does not seem to handle the fact that a select
# statement may not logically need to be evaluated for certain configurations
# (e.g., most targets can't be build for --cpu=web, so handling "web" in the
# cpu_select should notionally be unnecessary).
all_cpus = [
    "amd64",
    "roborio",
    "armhf",
    "cortex-m",
]

"""All of the CPUs we know about."""

"""A select wrapper for CPU architectures.

Args:
  values: A mapping from architecture names (as strings) to other things.
          Currently amd64, roborio, armhf, and cortex are recognized.
          'else' is also allowed as a default.
          'arm' is allowed instead of roborio, armhf, and cortex.
Returns a select which evaluates to the correct element of values.
"""

def cpu_select(values):
    if "arm" in values:
        new_values = {}
        for cpu in values:
            if cpu != "arm":
                new_values[cpu] = values[cpu]
        new_values["armhf"] = values["arm"]
        new_values["roborio"] = values["arm"]
        new_values["cortex-m"] = values["arm"]
        values = new_values
    for cpu in all_cpus:
        if cpu not in values:
            if "else" in values:
                values[cpu] = values["else"]
            else:
                fail("Need to handle %s CPUs!" % cpu, "values")
    for key in values:
        if key not in all_cpus and key != "else":
            fail("Not sure what a %s CPU is!" % key, "values")
    return select({
        "@//tools:cpu_k8": values["amd64"],
        "@//tools:cpu_roborio": values["roborio"],
        "@//tools:cpu_armhf": values["armhf"],
        "@//tools:cpu_cortex_m4f": values["cortex-m"],
        "@//tools:cpu_cortex_m4f_k22": values["cortex-m"],
        "@//tools:cpu_web": None,
    })

"""A select wrapper for address space sizes.

Args:
  values: A mapping from address space sizes (as strings) to other things.
Returns a select which evaluates to the correct element of values.
"""

def address_size_select(values):
    if "32" not in values:
        fail("Need to handle 32 bit addresses!", "values")
    if "64" not in values:
        fail("Need to handle 64 bit addresses!", "values")
    return select({
        "@//tools:cpu_k8": values["64"],
        "@//tools:cpu_roborio": values["32"],
        "@//tools:cpu_armhf": values["32"],
        "@//tools:cpu_cortex_m4f": values["32"],
        "@//tools:cpu_cortex_m4f_k22": values["32"],
        "@//tools:cpu_web": None,
    })

"""A select wrapper for compilers.

Args:
  values: A mapping from compiler names (as strings) to other things.
          Currently 'gcc' and 'clang' are recognized.
Returns a select which evaluates to the correct element of values.
"""

def compiler_select(values):
    if "gcc" not in values:
        fail("Need to handle gcc!", "values")
    if "clang" not in values:
        fail("Need to handle clang!", "values")
    return select({
        "@//tools:compiler_gcc": values["gcc"],
        "@//tools:compiler_clang": values["clang"],
        "@//tools:compiler_emscripten": None,
    })
