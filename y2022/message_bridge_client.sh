#!/bin/bash

while true;
do
  ping -c 1 pi1 -W 1 && break;
  sleep 1
done

echo Pinged

exec /home/admin/bin/message_bridge_client "$@"
