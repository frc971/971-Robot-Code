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
PYTHON_BIN=""
for path in ${PYTHONPATH//:/ }; do
  if [[ "$path" == *.runfiles/python3_9_x86_64-unknown-linux-gnu ]]; then
    PYTHON_BIN="$path"/bin/python3
    LD_LIBRARY_PATH=":${path}/lib"
    LD_LIBRARY_PATH+=":${path}/../gtk_runtime/lib/x86_64-linux-gnu"
    LD_LIBRARY_PATH+=":${path}/../gtk_runtime/usr/lib/x86_64-linux-gnu"
    LD_LIBRARY_PATH+=":${path}/../gtk_runtime/usr/lib"
    if [[ -e "${path}/../pip_deps_nvidia_nccl_cu12" ]]; then
      LD_LIBRARY_PATH+=":${path}/../pip_deps_nvidia_nccl_cu12/site-packages/nvidia/nccl/lib/"
    fi
    export LD_LIBRARY_PATH
    break
  fi
done

if [[ -z "$PYTHON_BIN" ]]; then
  echo "Could not find Python base path." >&2
  echo "More sophisticated logic may be needed." >&2
  exit 1
fi

export XLA_FLAGS=--xla_gpu_cuda_data_dir=/usr/lib/cuda

# Prevent Python from importing the host's installed packages.
exec "$PYTHON_BIN" -sS "$@"
