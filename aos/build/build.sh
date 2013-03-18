#!/bin/bash -e
#set -x

# This file should be called to build the code.
# Usage: build.sh platform main_file.gyp debug [action]

PLATFORM=$1
GYP_MAIN=$2
DEBUG=$3
ACTION=$4

export WIND_BASE=${WIND_BASE:-"/usr/local/powerpc-wrs-vxworks/wind_base"}

[ ${PLATFORM} == "crio" -o ${PLATFORM} == "atom" ] || ( echo Platform "(${PLATFORM})" must be '"crio" or "atom"'. ; exit 1 )
[ ${DEBUG} == "yes" -o ${DEBUG} == "no" ] || ( echo Debug "(${DEBUG})" must be '"yes" or "no"'. ; exit 1 )

AOS=`dirname $0`/..
NINJA_DIR=${AOS}/externals/ninja
NINJA=${NINJA_DIR}/ninja
# From chromium@154360:trunk/src/DEPS.
GYP_REVISION=1488
GYP_DIR=${AOS}/externals/gyp-${GYP_REVISION}
GYP=${GYP_DIR}/gyp

OUTDIR=${AOS}/../out_${PLATFORM}
BUILD_NINJA=${OUTDIR}/Default/build.ninja

[ -d ${NINJA_DIR} ] || git clone --branch release https://github.com/martine/ninja.git ${NINJA_DIR}
[ -x ${NINJA} ] || ${NINJA_DIR}/bootstrap.py
[ -d ${GYP_DIR} ] || ( svn co http://gyp.googlecode.com/svn/trunk -r ${GYP_REVISION} ${GYP_DIR} && patch -p1 -d ${GYP_DIR} < ${AOS}/externals/gyp.patch )
${AOS}/build/download_externals.sh

# The exciting quoting is so that it ends up with -DWHATEVER='"'`a command`'"'.
# The '"' at either end is so that it creates a string constant when expanded
#   in the C/C++ code.
COMMONFLAGS='-DLOG_SOURCENAME='"'\"'"'`basename $in`'"'\"' "
if [ ${PLATFORM} == crio ]; then
  COMMONFLAGS+='-DAOS_INITNAME=aos_init_function_`readlink -f $out | sed \"s/[\/.]/_/g\"` '
fi

if [[ "${ACTION}" != "clean" && ( ! -d ${OUTDIR} || -n \
  			"`find ${AOS}/.. -newer ${BUILD_NINJA} \( -name '*.gyp' -or -name '*.gypi' \)`" ) ]]; then
  ${GYP} \
    --check --depth=${AOS}/.. --no-circular-check -f ninja \
    -I${AOS}/build/aos.gypi -Goutput_dir=out_${PLATFORM} \
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
  rm -r ${OUTDIR}
else
  if [ "${ACTION}" != "deploy" -a "${ACTION}" != "tests" ]; then
					GYP_ACTION=${ACTION}
	else
					GYP_ACTION=
	fi
  ${NINJA} -C ${OUTDIR}/Default ${GYP_ACTION}
  case ${ACTION} in
    deploy)
      [ ${PLATFORM} == atom ] && \
        rsync --progress -r --rsync-path=/home/driver/bin/rsync \
        ${OUTDIR}/Default/outputs/* \
        driver@`${AOS}/build/get_ip fitpc`:/home/driver/robot_code/bin
      [ ${PLATFORM} == crio ] && \
        ncftpput `${AOS}/build/get_ip robot` / \
        ${OUTDIR}/Default/lib/FRC_UserProgram.out
      ;;
    tests)
      find ${OUTDIR}/Default/tests -executable -exec {} \;
      ;;
  esac
fi
