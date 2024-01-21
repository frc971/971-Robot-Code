#!/bin/bash

set -xeuo pipefail

HOSTNAME="$1"

# TODO<Jim>: Should probably add handling for imu hostname, too
if [[ ! "${HOSTNAME}" =~ ^orin-[0-9]*-[0-9]$ ]]; then
  echo "Invalid hostname ${HOSTNAME}, needs to be orin-[team#]-[orin#]"
  exit 1
fi

TEAM_NUMBER="$(echo ${HOSTNAME} | sed 's/orin-\(.*\)-.*/\1/')"
ORIN_NUMBER="$(echo ${HOSTNAME} | sed 's/orin-.*-\(.*\)/\1/')"
IP_BASE="$(echo ${TEAM_NUMBER} | sed 's/\(.*\)\(..\)/10.\1.\2/')"
IP="${IP_BASE}.$(( 100 + ${ORIN_NUMBER}))"

echo "Changing to team number ${TEAM_NUMBER}, IP ${IP}"

sed -i "s/^Address=.*$/Address=${IP}\/24/" /etc/systemd/network/eth0.network
sed -i "s/^Gateway=.*$/Gateway=${IP_BASE}.13/" /etc/systemd/network/eth0.network

echo "${HOSTNAME}" > /etc/hostname

# Make sure a 127.0.* entry exists to make things looking up localhost happy.
if grep '^127.0.1.1' /etc/hosts > /dev/null;
then
  sed -i "s/\(127\.0\.1\.1\t\).*$/\1${HOSTNAME}/" /etc/hosts
else
  echo -e "127.0.1.1\t${HOSTNAME}" >> /etc/hosts
fi

# Put correct team number in orin's IP addresses, or add them if needed
if grep '^10\.[0-9]*\.[0-9]*\.[0-9]*\s*orin-[0-9]*-[0-9] orin[0-9]$' /etc/hosts >/dev/null ;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.[0-9]*\s*orin-\)[0-9]*\(-[0-9] orin[0-9]\)\(.*\)$/${IP_BASE}\1${TEAM_NUMBER}\2\3/" /etc/hosts
else
  for i in {1..3}; do
      imu=""
      # Add imu name to orin3.  Put space in this string, since extra
      # spaces otherwise will make the above grep fail
      if [[ ${i} == 3 ]]; then
          imu=" imu"
      fi
    echo -e "${IP_BASE}.$(( i + 100 ))\torin-${TEAM_NUMBER}-${i} orin${i}${imu}" >> /etc/hosts
  done
fi

# Put correct team number in roborio's address, or add it if missing
if grep '^10\.[0-9]*\.[0-9]*\.2\s*roborio$' /etc/hosts >/dev/null;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.2\s*roborio\)$/${IP_BASE}\1/" /etc/hosts
else
  echo -e "${IP_BASE}.2\troborio" >> /etc/hosts
fi
