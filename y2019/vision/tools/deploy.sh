#!/bin/bash
set -e

BRANCH=$(git rev-parse --symbolic-full-name --abbrev-ref HEAD)
if [[ "$BRANCH" != "master" ]]; then
  read -p "Not on master, deploy anyway (y/n) " ANSWER
  if [[ $ANSWER =~ ^[Yy]$ ]]; then
    echo "Master check overridden, deploying anyway"
  else
    echo "Cancelling deploy"
    exit 1
  fi
else
  echo "On master, deploying"
fi

echo "Building executables"
readonly BAZEL_OPTIONS="-c opt --cpu=armhf-debian"
readonly BAZEL_BIN="$(bazel info ${BAZEL_OPTIONS} bazel-bin)"
readonly TARGET_DIR=/media/$USER/JEVOIS

bazel build ${BAZEL_OPTIONS} \
    //y2019/vision:target_sender \
    //y2019/vision:serial_waiter

if [ ! -d "${TARGET_DIR}" ]
then
  echo "Mount jevois at ${TARGET_DIR} ..."
  ./jevois-cmd usbsd
fi

echo "Waiting for fs ..."
while [ ! -d "${TARGET_DIR}" ]
do
  sleep 1
done
echo "OK"

echo "Copying files ..."
cp ./austin_cam.sh "${TARGET_DIR}"/
cp ./launch.sh "${TARGET_DIR}"/deploy/

cp "${BAZEL_BIN}/y2019/vision/target_sender" \
  "${BAZEL_BIN}/y2019/vision/serial_waiter" \
  "${TARGET_DIR}"/deploy/

echo "Unmount sd card ..."
umount "${TARGET_DIR}"
echo "OK"

echo "Rebooting Jevois."
./jevois-cmd restart
