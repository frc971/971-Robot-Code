#!/bin/bash
#set -x

set -e

# This file should be called to build the code.
# Usage: build.sh platform main_file.gyp debug [action]...

PLATFORM=$1
GYP_MAIN=$2
DEBUG=$3
ACTION=$4

shift 3
shift || true # We might not have a 4th argument if ACTION is empty.

export WIND_BASE=${WIND_BASE:-"/usr/local/powerpc-wrs-vxworks/wind_base"}

[ "${PLATFORM}" == "crio" -o "${PLATFORM}" == "atom" ] || ( echo Platform "(${PLATFORM})" must be '"crio" or "atom"'. ; exit 1 )
[ "${DEBUG}" == "yes" -o "${DEBUG}" == "no" ] || ( echo Debug "(${DEBUG})" must be '"yes" or "no"'. ; exit 1 )

AOS=`dirname $0`/..

OUTDIR=${AOS}/../output/${PLATFORM}
BUILD_NINJA=${OUTDIR}/build.ninja

${AOS}/build/download_externals.sh
. $(dirname $0)/tools_config

# The exciting quoting is so that it ends up with -DWHATEVER='"'`a command`'"'.
# The '"' at either end is so that it creates a string constant when expanded
#   in the C/C++ code.
COMMONFLAGS='-DLOG_SOURCENAME='"'\"'"'`basename $in`'"'\"' "

if [[ "${ACTION}" != "clean" && ( ! -d ${OUTDIR} || -n \
  			"`find ${AOS}/.. -newer ${BUILD_NINJA} \( -name '*.gyp' -or -name '*.gypi' \)`" ) ]]; then
  # This is a gyp "file" that we pipe into gyp so that it will put the output
  # in a directory named what we want where we want it.
  GYP_INCLUDE=$(cat <<END
{
  'target_defaults': {
    'configurations': {
	  '${PLATFORM}': {}
    }
  }
}
END
)
  echo "${GYP_INCLUDE}" | ${GYP} \
      --check --depth=${AOS}/.. --no-circular-check -f ninja \
      -I${AOS}/build/aos.gypi -I/dev/stdin -Goutput_dir=output \
      -DOS=${PLATFORM} -DWIND_BASE=${WIND_BASE} -DDEBUG=${DEBUG} \
      ${GYP_MAIN}
  # Have to substitute "command = $compiler" so that it doesn't try to
  #   substitute them in the linker commands, where it doesn't work.
  sed -i "s:command = \$cc:\\0 ${COMMONFLAGS}:g ; \
    s:command = \$cxx:\\0 ${COMMONFLAGS}:g" \
    ${BUILD_NINJA}
  if [ ${PLATFORM} == crio ]; then
    sed -i 's/nm -gD/nm/g' ${BUILD_NINJA}
  fi
fi

if [ "${ACTION}" == "clean" ]; then
  rm -r ${OUTDIR} || true
else
  if [ "${ACTION}" != "deploy" -a "${ACTION}" != "tests" -a "${ACTION}" != "redeploy" ]; then
    NINJA_ACTION=${ACTION}
  else
    NINJA_ACTION=
  fi
  ${NINJA} -C ${OUTDIR} ${NINJA_ACTION} "$@"
  if [[ ${ACTION} == deploy || ${ACTION} == redeploy ]]; then
    [ ${PLATFORM} == atom ] && \
      rsync --progress -c -r \
        ${OUTDIR}/outputs/* \
        driver@`${AOS}/build/get_ip fitpc`:/home/driver/robot_code/bin
	  ssh driver@`${AOS}/build/get_ip fitpc` "sync; sync; sync"
    [ ${PLATFORM} == crio ] && \
      ncftpput `${AOS}/build/get_ip robot` / \
      ${OUTDIR}/lib/FRC_UserProgram.out
  fi
  if [[ ${ACTION} == redeploy ]]; then
    if [[ ${PLATFORM} != crio ]]; then
      echo "Platform ${PLATFORM} does not support redeploy." 1>&2
      exit 1
    fi
    ${OUTDIR}/../out_atom/outputs/netconsole <<"END"
unld "FRC_UserProgram.out"
ld < FRC_UserProgram.out
END
  fi
  if [[ ${ACTION} == tests ]]; then
    find ${OUTDIR}/tests -executable -exec ${AOS}/build/run_test.sh {} \;
  fi
fi
