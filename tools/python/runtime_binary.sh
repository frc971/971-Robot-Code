#!/bin/bash

set -e
set -u
set -o pipefail

# We disable writing .pyc files here so that the invocation is more
# deterministic. If we get a corrupted .pyc file (for some reason) in the
# .runfiles directory the corresponding Python invocation would crash with an
# EOFError. You can try this by calling truncate(1) on a .pyc file and running
# your Python script.
# In the bazel sandbox none of the .pyc files are preserved anyway.
# Sandboxing also means that Python's entire standard library got cached which
# normally doesn't happen. That can lead to higher memory usage during the
# individual build steps.
export PYTHONDONTWRITEBYTECODE=1

# Find the path that contains the Python runtime. It's not always obvious. For
# example in a genrule the Python runtime is in the runfiles folder of the
# tool, not of the genrule.
# TODO(philipp): Is there a better way to do this?
BASE_PATH=""
for path in ${PYTHONPATH//:/ }; do
  if [[ "$path" == *.runfiles/python_repo ]]; then
    BASE_PATH="$path"
    export LD_LIBRARY_PATH="$path"/lib/x86_64-linux-gnu:"$path"/usr/lib:"$path"/usr/lib/x86_64-linux-gnu
    break
  fi
done

if [[ -z "$BASE_PATH" ]]; then
  echo "Could not find Python base path." >&2
  echo "More sophisticated logic may be needed." >&2
  exit 1
fi

export LD_LIBRARY_PATH="${BASE_PATH}/usr/lib/lapack:${BASE_PATH}/usr/lib/libblas:${BASE_PATH}/usr/lib/x86_64-linux-gnu"

if head -n 1 "$1" | grep -q python3; then
  exec "$BASE_PATH"/usr/bin/python3 "$@"
else
  exec "$BASE_PATH"/usr/bin/python2 "$@"
fi
