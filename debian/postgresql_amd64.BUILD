load("@bazel_skylib//rules:write_file.bzl", "write_file")

TEMPLATE = """\
#!/bin/bash

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

add_ld_library_path_for() {
  local file="$1"
  local dir
  local resolved_file
  if ! resolved_file="$(rlocation "postgresql_amd64/$file")"; then
    echo "Couldn't find file postgresql_amd64/${file}" >&2
    exit 1
  fi
  dir="$(dirname "${resolved_file}")"
  export LD_LIBRARY_PATH="${dir}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
}

add_ld_library_path_for usr/lib/x86_64-linux-gnu/libbsd.so.0.11.3
add_ld_library_path_for lib/x86_64-linux-gnu/libreadline.so.8.1

exec $(rlocation postgresql_amd64/usr/lib/postgresql/13/bin/%s) "$@"
"""

[(
    write_file(
        name = "generate_%s_wrapper" % binary,
        out = "%s.sh" % binary,
        content = [TEMPLATE % binary],
    ),
    sh_binary(
        name = binary,
        srcs = ["%s.sh" % binary],
        data = glob([
            "usr/lib/**/*",
            "lib/**/*",
        ]),
        visibility = ["//visibility:public"],
        deps = [
            "@bazel_tools//tools/bash/runfiles",
        ],
    ),
) for binary in (
    "postgres",
    "initdb",
)]
