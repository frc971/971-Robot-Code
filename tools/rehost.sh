#!/bin/bash
#
# This script makes it easy to rehost dependencies on software.frc971.org so
# we are self hosted everywhere consistently.

# ./rehost.sh github.com/bazelbuild/rules_nodejs/releases/download/4.4.6/rules_nodejs-4.4.6.tar.gz
set -e

readonly LOCALPATH="$(basename "${1}")"

# Strip off any http/https prefix to make things easy below.
STRIPPED_URL="${1#"https://"}"
STRIPPED_URL="${STRIPPED_URL#"http://"}"

curl -L "https://${STRIPPED_URL}" -o "${LOCALPATH}"

echo "https://software.frc971.org/Build-Dependencies/${STRIPPED_URL}"

readonly TARGET_DIR="/data/files/frc971/Build-Dependencies"

ssh software.frc971.org mkdir --m=775 -p "$(dirname "${TARGET_DIR}/${STRIPPED_URL}")"
rsync --progress -v --ignore-existing "${LOCALPATH}" software.frc971.org:"${TARGET_DIR}/${STRIPPED_URL}"
ssh software.frc971.org "find ${TARGET_DIR}/ -type f ! -user www-data ! -group www-data -exec chmod 444 {} \; && find ${TARGET_DIR}/ -type d ! -user www-data ! -group www-data -exec chmod 775 {} \; && find ${TARGET_DIR}/ -type d ! -user www-data ! -group www-data -exec chown www-data:www-data {} \;"
