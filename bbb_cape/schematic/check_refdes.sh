#!/bin/bash

if [[ $# -lt 1 ]]; then
	find $(dirname $0) \( -name '*.sch' -or -name '*.sym' \) -exec $0 {} \;
	exit $?
fi

FILE=$1

DUPLICATED=$(grep -F 'refdes=' "${FILE}" | sort | uniq -d)

if [[ -n ${DUPLICATED} ]]; then
	echo Duplicated lines in ${FILE}:
	echo ${DUPLICATED}
fi
