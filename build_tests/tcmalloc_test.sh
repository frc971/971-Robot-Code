set -e
set -u

OUTPUT=$(TCMALLOC_LARGE_ALLOC_REPORT_THRESHOLD=10 build_tests/tcmalloc_build_test_binary 2>&1)

if [[ -z "${OUTPUT}" ]]; then
  echo 'Empty output!' >&2
  exit 1
fi

PATTERN='tcmalloc: large alloc [0-9]+ bytes =='

if [[ ! "${OUTPUT}" =~ ${PATTERN} ]]; then
  echo 'Unexpected output:' >&2
  echo "${OUTPUT}" >&2
  exit 1
fi
