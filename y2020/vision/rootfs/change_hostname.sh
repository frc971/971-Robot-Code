#!/bin/bash

set -euo pipefail

HOSTNAME="$1"

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

if grep /etc/hosts '^10\.[0-9]*\.[0-9]*\.[0-9]*\s*pi-[0-9]*-[0-9] pi[0-9]$' /etc/hosts >/dev/null ;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.[0-9]*\s*pi-\)[0-9]*\(-[0-9] pi[0-9]\)$/${IP_BASE}\1${TEAM_NUMBER}\2/" /etc/hosts
else
  for i in {1..6}; do
    echo -e "${IP_BASE}.$(( i + 100 ))\tpi-${TEAM_NUMBER}-${i} pi${i}" >> /etc/hosts
  done
fi

if grep '^10\.[0-9]*\.[0-9]*\.2\s*roborio$' /etc/hosts >/dev/null;
then
  sed -i "s/^10\.[0-9]*\.[0-9]*\(\.2\s*roborio\)$/${IP_BASE}\1/" /etc/hosts
else
  echo -e "${IP_BASE}.2\troborio" >> /etc/hosts
fi

