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

JEVOIS_DEV="/dev/null"
for dev in /dev/ttyACM*; do
  if udevadm info -a -n "${dev}" | grep "JeVois-A33 Smart Camera" -q;
  then
    JEVOIS_DEV="${dev}"
  fi
done

if [[ "${JEVOIS_DEV}" == "/dev/null" ]];
then
  echo "Can't find jevois"
  exit 1;
fi;

if ! mount | grep "${TARGET_DIR}" -q
then
  echo "Mount jevois at ${TARGET_DIR} ..."
  ./jevois-cmd -d "${JEVOIS_DEV}" usbsd
fi

sleep 5

JEVOIS_SD=
for SD in /dev/sd[a-z]; do
  if udevadm info -a -n "${SD}" | grep JeVois -q; then
    echo "Jevois at ${SD}"
    JEVOIS_SD="${SD}"
    break
  fi
done
if [[ -z ${JEVOIS_SD} ]]; then
  echo "Failed to find Jevois. Stopping now"
  exit 1
fi

if ! mount | grep "${TARGET_DIR}" -q
then
  sudo mkdir -p "${TARGET_DIR}"

  sudo mount "${JEVOIS_SD}" "${TARGET_DIR}"
fi

echo "Waiting for fs ..."
while [ ! -d "${TARGET_DIR}" ]
do
  sleep 1
done
echo "OK"

echo "Copying files ..."
sudo cp ./austin_cam.sh "${TARGET_DIR}"/
sudo cp ./launch.sh "${TARGET_DIR}"/deploy/

(echo "git log"; git log -1; echo "git status"; git status) > /tmp/jevois_deploy_version

sudo cp /tmp/jevois_deploy_version "${TARGET_DIR}"/version

sudo cp "${BAZEL_BIN}/y2019/vision/target_sender" \
  "${BAZEL_BIN}/y2019/vision/serial_waiter" \
  "${TARGET_DIR}"/deploy/

echo "Unmount sd card ..."
sudo umount "${TARGET_DIR}"
echo "OK"

echo "Rebooting Jevois."
./jevois-cmd -d "${JEVOIS_DEV}" restart
