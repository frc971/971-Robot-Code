load("@bazel_skylib//rules:write_file.bzl", "write_file")

write_file(
    name = "generate_wrapper",
    out = "wrapped_bin/Xvfb.sh",
    content = ["""\
#!/bin/bash

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; set +e; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---


runfiles_export_envvars

export LD_LIBRARY_PATH="${RUNFILES_DIR}/xvfb_amd64/usr/lib/x86_64-linux-gnu"
export LD_LIBRARY_PATH+=":${RUNFILES_DIR}/xvfb_amd64/lib/x86_64-linux-gnu"

exec "${RUNFILES_DIR}/xvfb_amd64/usr/bin/Xvfb" "$@"
"""],
    is_executable = True,
)

sh_binary(
    name = "wrapped_bin/Xvfb",
    srcs = ["wrapped_bin/Xvfb.sh"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
    data = glob([
        "usr/lib/**/*",
        "lib/**/*",
        "usr/bin/*",
    ]),
    visibility = ["//visibility:public"],
)
