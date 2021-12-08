#!/bin/bash

set -euo pipefail

result=0
verify() {
  local thisresult=0
  local family
  local address
  family=$(echo "$output" | grep -Po '\] Family [^ ]+' | cut -f3 -d' ')
  address=$(echo "$output" | grep -Po '\] Address [^ ]+' | cut -f3 -d' ')
  if [[ "${family}" != "${1}" ]]; then
    echo "Expected family ${1}, got ${family}" >&2
    thisresult=1
    result=1
  fi
  if [[ ! "${address}" =~ ${2} ]]; then
    echo "Expected address ${2}, got ${address}" >&2
    thisresult=1
    result=1
  fi
  return $thisresult
}

run_test() {
   local has_ipv6
   has_ipv6="${1}"
   export has_ipv6
   shift
   LD_PRELOAD="${SHIM}" "${BINARY}" --host=localhost "$@" 2>&1
}

BINARY="$1"
SHIM="$2"

output=$(run_test y)
verify AF_INET6 "(::ffff:127.0.0.1|::)" || echo "IPv6 allowed with no arguments failed" >&2

output=$(run_test n)
verify AF_INET "127\\.0\\.0\\.1" || echo "IPv6 disallowed with no arguments failed" >&2

output=$(run_test y --disable_ipv6)
verify AF_INET "127\\.0\\.0\\.1" || echo "IPv6 allowed with --disable_ipv6 failed" >&2

exit $result
