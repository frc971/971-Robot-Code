#!/bin/bash
#
# This script makes it easy to rehost dependencies on software.frc971.org so
# we are self hosted everywhere consistently.

# ./rehost.sh github.com/bazelbuild/rules_nodejs/releases/download/4.4.6/rules_nodejs-4.4.6.tar.gz

readonly LOCALPATH="$(basename "${1}")"

set -e
curl -L "https://${1}" -o "${LOCALPATH}"

echo "https://software.frc971.org/Build-Dependencies/${1}"

ssh software.frc971.org mkdir -p "$(dirname "/data/files/frc971/Build-Dependencies/${1}")"
rsync --progress -v --ignore-existing "${LOCALPATH}" software.frc971.org:"/data/files/frc971/Build-Dependencies/${1}"
