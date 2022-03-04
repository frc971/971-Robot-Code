#!/bin/bash

set -xeuo pipefail

HOSTNAME="$1"

# TODO<Jim>: Should probably add handling for imu hostname, too
if [[ ! "${HOSTNAME}" =~ ^pi-[0-9]*-[0-9]$ ]]; then
  echo "Invalid hostname ${HOSTNAME}, needs to be pi-[team#]-[pi#]"
  exit 1
fi

TEAM_NUMBER="$(echo ${HOSTNAME} | sed 's/pi-\(.*\)-.*/\1/')"
PI_NUMBER="$(echo ${HOSTNAME} | sed 's/pi-.*-\(.*\)/\1/')"
IP_BASE="$(echo ${TEAM_NUMBER} | sed 's/\(.*\)\(..\)/10.\1.\2/')"
IP="${IP_BASE}.$(( 100 + ${PI_NUMBER}))"

echo "Changing to team number ${TEAM_NUMBER}, IP ${IP}"

sed -i "s/^static ip_address=.*$/static ip_address=${IP}\/24/" /etc/dhcpcd.conf

sed -i "s/\(127\.0\.1\.1\t\).*$/\1${HOSTNAME}/" /etc/hosts

echo "${HOSTNAME}" > /etc/hostname

# Put corret team number in pi's IP addresses, or add them if needed
if grep '^10\.[0-9]*\.[0-9]*\.[0-9]*\s*pi-[0-9]*-[0-9] pi[0-9]$' /etc/hosts >/dev/null ;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.[0-9]*\s*pi-\)[0-9]*\(-[0-9] pi[0-9]\)$/${IP_BASE}\1${TEAM_NUMBER}\2/" /etc/hosts
else
  for i in {1..6}; do
    echo -e "${IP_BASE}.$(( i + 100 ))\tpi-${TEAM_NUMBER}-${i} pi${i}" >> /etc/hosts
  done
fi

# Put corret team number in roborio's address, or add it if missing
if grep '^10\.[0-9]*\.[0-9]*\.2\s*roborio$' /etc/hosts >/dev/null;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.2\s*roborio\)$/${IP_BASE}\1/" /etc/hosts
else
  echo -e "${IP_BASE}.2\troborio" >> /etc/hosts
fi

# Put corret team number in logger's address, or add it if missing
if grep '^10\.[0-9]*\.[0-9]*\.13\s*logger$' /etc/hosts >/dev/null;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.13\s*logger\)$/${IP_BASE}\1/" /etc/hosts
else
  echo -e "${IP_BASE}.13\tlogger" >> /etc/hosts
fi

# Put corret team number in imu's address, or add it if missing
if grep '^10\.[0-9]*\.[0-9]*\.105\s.*\s*imu$' /etc/hosts >/dev/null;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.[0-9]*\s*pi-\)[0-9]*\(-[0-9] pi5 imu\)$/${IP_BASE}\1${TEAM_NUMBER}\2/" /etc/hosts
else
  if grep '^10\.[0-9]*\.[0-9]*\.105\s*pi-[0-9]*-[0-9]*\s*pi5$' /etc/hosts
  then
    sed -i "s/^10\.[0-9]*\.[0-9]*\(\.[0-9]*\s*pi-\)[0-9]*\(-[0-9] pi5\)$/${IP_BASE}\1${TEAM_NUMBER}\2 imu/" /etc/hosts
  else
    echo -e "${IP_BASE}.105\tpi-${TEAM_NUMBER}-5 pi5 imu" >> /etc/hosts
  fi
fi
