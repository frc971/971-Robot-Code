#!/bin/bash
#set -x

set -e

# This file should be called to build the code.
# Usage: build.sh platform main_file.gyp debug [action]...

PLATFORM=$1
GYP_MAIN=$2
DEBUG=$3
OUT_NAME=$4
ACTION=$5

shift 4
shift || true # We might not have a 5th argument if ACTION is empty.

export WIND_BASE=${WIND_BASE:-"/usr/local/powerpc-wrs-vxworks/wind_base"}

[ "${PLATFORM}" == "crio" -o "${PLATFORM}" == "linux" -o "${PLATFORM}" == "linux-amd64" ] || ( echo Platform "(${PLATFORM})" must be '"crio", "linux", or "linux-amd64"'. ; exit 1 )
[ "${DEBUG}" == "yes" -o "${DEBUG}" == "no" ] || ( echo Debug "(${DEBUG})" must be '"yes" or "no"'. ; exit 1 )

AOS=`dirname $0`/..

OUTDIR=${AOS}/../output/${OUT_NAME}
BUILD_NINJA=${OUTDIR}/build.ninja

${AOS}/build/download_externals.sh arm
${AOS}/build/download_externals.sh amd64
. $(dirname $0)/tools_config

# The exciting quoting is so that it ends up with -DWHATEVER='"'`a command`'"'.
# The '"' at either end is so that it creates a string constant when expanded
#   in the C/C++ code.
COMMONFLAGS='-DLOG_SOURCENAME='"'\"'"'`basename $in`'"'\"' "

if [[ "${ACTION}" != "clean" && ( ! -d ${OUTDIR} || -n \
  			"`find ${AOS}/.. -newer ${BUILD_NINJA} \( -name '*.gyp' -or -name '*.gypi' \)`" ) ]]; then
  echo 'Running gyp...' 1>&2
  # This is a gyp "file" that we pipe into gyp so that it will put the output
  # in a directory named what we want where we want it.
  GYP_INCLUDE=$(cat <<END
{
  'target_defaults': {
    'configurations': {
	  '${OUT_NAME}': {}
    }
  }
}
END
)
  echo "${GYP_INCLUDE}" | ${GYP} \
      --check --depth=${AOS}/.. --no-circular-check -f ninja \
      -I${AOS}/build/aos.gypi -I/dev/stdin -Goutput_dir=output \
      -DOS=$(echo ${PLATFORM} | sed 's/-.*//g') -DPLATFORM=${PLATFORM} \
      -DWIND_BASE=${WIND_BASE} -DDEBUG=${DEBUG} \
      ${GYP_MAIN}
  # Have to substitute "command = $compiler" so that it doesn't try to
  #   substitute them in the linker commands, where it doesn't work.
  sed -i "s:command = \$cc:\\0 ${COMMONFLAGS}:g ; \
    s:command = \$cxx:\\0 ${COMMONFLAGS}:g" \
    ${BUILD_NINJA}
  if [ ${PLATFORM} == crio ]; then
    sed -i 's/nm -gD/nm/g' ${BUILD_NINJA}
  fi
  echo 'Done running gyp.' 1>&2
fi

if [ "${ACTION}" == "clean" ]; then
  rm -r ${OUTDIR} || true
else
  if [ "${ACTION}" != "deploy" -a "${ACTION}" != "tests" ]; then
    NINJA_ACTION=${ACTION}
  else
    NINJA_ACTION=
  fi
  ${NINJA} -C ${OUTDIR} ${NINJA_ACTION} "$@"
  if [[ ${ACTION} == deploy ]]; then
    [[ ${PLATFORM} == linux ]] && \
      rsync --progress -c -r \
        ${OUTDIR}/outputs/* \
        driver@`${AOS}/build/get_ip prime`:/home/driver/robot_code/bin
	  ssh driver@`${AOS}/build/get_ip prime` "sync; sync; sync"
    [ ${PLATFORM} == crio ] && \
      ncftpput `${AOS}/build/get_ip robot` / \
      ${OUTDIR}/lib/FRC_UserProgram.out
  fi
  if [[ ${ACTION} == tests ]]; then
    find ${OUTDIR}/tests -executable -exec {} \;
  fi
fi
