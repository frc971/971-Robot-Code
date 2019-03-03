#!/bin/sh
echo "Building executables"
readonly BAZEL_OPTIONS="-c opt --cpu=armhf-debian"
readonly BAZEL_BIN="$(bazel info ${BAZEL_OPTIONS} bazel-bin)"

bazel build ${BAZEL_OPTIONS} \
    //y2019/vision:target_sender \
    //y2019/vision:serial_waiter

if [ ! -d /media/$USER/JEVOIS ]
then
  echo "Mount jevois at /media/$USER/JEVOIS ..."
  ./jevois-cmd usbsd
fi

echo "Waiting for fs ..."
while [ ! -d /media/$USER/JEVOIS ]
do
  sleep 1
done
echo "OK"

echo "Copying files ..."
cp ./austin_cam.sh /media/$USER/JEVOIS/
cp ./launch.sh /media/$USER/JEVOIS/deploy/

cp "${BAZEL_BIN}/y2019/vision/target_sender" \
  "${BAZEL_BIN}/y2019/vision/serial_waiter" \
  /media/$USER/JEVOIS/deploy/

echo "Unmount sd card ..."
umount /media/$USER/JEVOIS
echo "OK"

echo "Rebooting Jevois."
./jevois-cmd restart
