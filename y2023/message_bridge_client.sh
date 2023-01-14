#!/bin/bash

while true;
do
  ping -c 1 pi1 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 pi2 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 pi3 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 pi4 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 pi5 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 pi6 -W 1 && break;
  sleep 1
done
while true;
do
  ping -c 1 roborio -W 1 && break;
  sleep 1
done

echo Pinged

exec message_bridge_client "$@"
