#!/bin/bash

# --- begin runfiles.bash initialization ---
# Copy-pasted from Bazel's Bash runfiles library (tools/bash/runfiles/runfiles.bash).
set -euo pipefail
if [[ ! -d "${RUNFILES_DIR:-/dev/null}" && ! -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
  if [[ -f "$0.runfiles_manifest" ]]; then
    export RUNFILES_MANIFEST_FILE="$0.runfiles_manifest"
  elif [[ -f "$0.runfiles/MANIFEST" ]]; then
    export RUNFILES_MANIFEST_FILE="$0.runfiles/MANIFEST"
  elif [[ -f "$0.runfiles/bazel_tools/tools/bash/runfiles/runfiles.bash" ]]; then
    export RUNFILES_DIR="$0.runfiles"
  fi
fi
if [[ -f "${RUNFILES_DIR:-/dev/null}/bazel_tools/tools/bash/runfiles/runfiles.bash" ]]; then
  source "${RUNFILES_DIR}/bazel_tools/tools/bash/runfiles/runfiles.bash"
elif [[ -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
  source "$(grep -m1 "^bazel_tools/tools/bash/runfiles/runfiles.bash " \
            "$RUNFILES_MANIFEST_FILE" | cut -d ' ' -f 2-)"
else
  echo >&2 "ERROR: cannot find @bazel_tools//tools/bash/runfiles:runfiles.bash"
  exit 1
fi
# --- end runfiles.bash initialization ---



# Copied from http://trac.ffmpeg.org/wiki/Create%20a%20mosaic%20out%20of%20several%20input%20videos

$(rlocation ffmpeg/ffmpeg)\
	-i $1 -i $2 -i $3 -i $4\
	-filter_complex "
		nullsrc=size=2560x1440 [base];
		[0:v] setpts=PTS-STARTPTS, scale=1280x720 [upperleft];
		[1:v] setpts=PTS-STARTPTS, scale=1280x720 [upperright];
		[2:v] setpts=PTS-STARTPTS, scale=1280x720 [lowerleft];
		[3:v] setpts=PTS-STARTPTS, scale=1280x720 [lowerright];
		[base][upperleft] overlay=shortest=1 [tmp1];
		[tmp1][upperright] overlay=shortest=1:x=1280 [tmp2];
		[tmp2][lowerleft] overlay=shortest=1:y=720 [tmp3];
		[tmp3][lowerright] overlay=shortest=1:x=1280:y=720
	" \
	-c:v libx264 ~/video_tiler_out.mp4
