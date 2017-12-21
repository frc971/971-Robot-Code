#!/usr/bin/env bash

set -e
set -u

readonly EXTRA_USER=user
readonly EXTRA_STORAGE=/home/"${EXTRA_USER}"

if ! grep -q "$EXTRA_STORAGE" /etc/passwd; then
  PASSWORD="$(echo "$EXTRA_USER" | mkpasswd -s)"
  useradd \
      --home="$EXTRA_STORAGE" -M \
      --password="$PASSWORD" \
      --shell=/bin/bash \
      "$EXTRA_USER"
  chown "$EXTRA_USER:$EXTRA_USER" "$EXTRA_STORAGE"
fi

usermod -a -G sudo "$EXTRA_USER"
