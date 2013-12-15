#!/bin/bash

set -e

AOS=$(readlink -f $(dirname $0)/..)
. $(dirname $0)/tools_config
COMPILED=${EXTERNALS}/../compiled-arm

export CC=arm-linux-gnueabihf-gcc-4.7
export CXX=arm-linux-gnueabihf-g++-4.7
export CFLAGS=-mcpu="cortex-a8 -mfpu=neon"
export CXXFLAGS=-mcpu="cortex-a8 -mfpu=neon"
export OBJDUMP=arm-linux-gnueabihf-objdump
# Flags that should get passed to all configure scripts.
# Some of them need to set LDFLAGS separately to work around stupid configure
# scripts, so we can't just set that here.
CONFIGURE_FLAGS="--host=arm-linux-gnueabihf CC=${CC} CXX=${CXX} CFLAGS=\"${CFLAGS}\" CXXFLAGS=\"${CXXFLAGS}\" OBJDUMP=${OBJDUMP}"

TMPDIR=/tmp/$$-aos-tmpdir
mkdir -p ${EXTERNALS}
mkdir -p ${COMPILED}

# get and build ninja
[ -d ${NINJA_DIR} ] || git clone --branch ${NINJA_RELEASE} https://github.com/martine/ninja.git ${NINJA_DIR}
[ -x ${NINJA} ] || env -i "PATH=$PATH" ${NINJA_DIR}/bootstrap.py

# get gyp
[ -d ${GYP_DIR} ] || ( svn co http://gyp.googlecode.com/svn/trunk -r ${GYP_REVISION} ${GYP_DIR} && patch -p1 -d ${GYP_DIR} < ${AOS}/externals/gyp.patch )

# get gccdist
GCCDIST=${EXTERNALS}/gccdist
[ -f ${GCCDIST}.zip ] || wget ftp://ftp.ni.com/pub/devzone/tut/updated_vxworks63gccdist.zip -O ${GCCDIST}.zip
[ -d ${GCCDIST} ] || ( cd ${EXTERNALS} && unzip -q ${GCCDIST}.zip )

# get eigen
EIGEN_VERSION=3.1.3
EIGEN_DIR=${EXTERNALS}/eigen-${EIGEN_VERSION}
[ -f ${EIGEN_DIR}.tar.bz2 ] || wget http://bitbucket.org/eigen/eigen/get/${EIGEN_VERSION}.tar.bz2 -O ${EIGEN_DIR}.tar.bz2
[ -d ${EIGEN_DIR} ] || ( mkdir ${EIGEN_DIR} && tar --strip-components=1 -C ${EIGEN_DIR} -xf ${EIGEN_DIR}.tar.bz2 )

# get the javacv binaries
JAVACV_VERSION=0.2
JAVACV_DIR=${EXTERNALS}/javacv-bin
JAVACV_ZIP=${EXTERNALS}/javacv-${JAVACV_VERSION}-bin.zip
[ -f ${JAVACV_ZIP} ] || wget http://javacv.googlecode.com/files/javacv-${JAVACV_VERSION}-bin.zip -O ${JAVACV_ZIP}
[ -d ${JAVACV_DIR} ] || ( cd ${EXTERNALS} && unzip ${JAVACV_ZIP} )

# get the simple one-jar template jar
ONEJAR_VERSION=0.97
ONEJAR_JAR=${EXTERNALS}/one-jar-boot-${ONEJAR_VERSION}.jar
[ -f ${ONEJAR_JAR} ] || wget http://sourceforge.net/projects/one-jar/files/one-jar/one-jar-${ONEJAR_VERSION}/one-jar-boot-${ONEJAR_VERSION}.jar/download -O ${ONEJAR_JAR}

# get and build libjpeg
LIBJPEG_VERSION=8d
LIBJPEG_DIR=${COMPILED}/jpeg-${LIBJPEG_VERSION}
# NOTE: this directory ends up in #include names
LIBJPEG_PREFIX=${COMPILED}/libjpeg
LIBJPEG_LIB=${LIBJPEG_PREFIX}/lib/libjpeg.a
LIBJPEG_TAR=${EXTERNALS}/jpegsrc.v${LIBJPEG_VERSION}.tar.gz
[ -f ${LIBJPEG_TAR} ] || wget http://www.ijg.org/files/jpegsrc.v${LIBJPEG_VERSION}.tar.gz -O ${LIBJPEG_TAR}
[ -d ${LIBJPEG_DIR} ] || ( mkdir ${LIBJPEG_DIR} && tar --strip-components=1 -C ${LIBJPEG_DIR} -xf ${LIBJPEG_TAR} )
[ -f ${LIBJPEG_LIB} ] || bash -c \
	"cd ${LIBJPEG_DIR} && ./configure --disable-shared \
	${CONFIGURE_FLAGS} --prefix=`readlink -f ${LIBJPEG_PREFIX}` \
	&& make && make install"

# get gtest
GTEST_VERSION=1.6.0
GTEST_DIR=${EXTERNALS}/gtest-${GTEST_VERSION}
GTEST_ZIP=${EXTERNALS}/gtest-${GTEST_VERSION}.zip
[ -f ${GTEST_ZIP} ] || wget http://googletest.googlecode.com/files/gtest-${GTEST_VERSION}.zip -O ${GTEST_ZIP}
[ -d ${GTEST_DIR} ] || ( unzip ${GTEST_ZIP} -d ${TMPDIR} && mv ${TMPDIR}/gtest-${GTEST_VERSION} ${GTEST_DIR} && cd ${GTEST_DIR} && patch -p1 < ${AOS}/externals/gtest.patch )

# get and build ctemplate
# This is the next revision after the 2.2 release and it only adds spaces to
# make gcc 4.7 with --std=c++11 happy (user-defined string literals...).
CTEMPLATE_VERSION=129
CTEMPLATE_TAR=${EXTERNALS}/ctemplate-${CTEMPLATE_VERSION}.tar.gz
CTEMPLATE_DIR=${COMPILED}/ctemplate-${CTEMPLATE_VERSION}
CTEMPLATE_PREFIX=${CTEMPLATE_DIR}-prefix
CTEMPLATE_LIB=${CTEMPLATE_PREFIX}/lib/libctemplate.a
CTEMPLATE_URL=http://ctemplate.googlecode.com
if [[ "${CTEMPLATE_VERSION}" =~ /\./ ]]; then
	CTEMPLATE_URL=${CTEMPLATE_URL}/files/ctemplate-${CTEMPLATE_VERSION}.tar.gz
	[ -f ${CTEMPLATE_TAR} ] || \
		wget ${CTEMPLATE_URL} -O ${CTEMPLATE_TAR}
	[ -d ${CTEMPLATE_DIR} ] || ( mkdir ${CTEMPLATE_DIR} && tar \
		--strip-components=1 -C ${CTEMPLATE_DIR} -xf ${CTEMPLATE_TAR} )
else
	CTEMPLATE_URL=${CTEMPLATE_URL}/svn/trunk
	[ -d ${CTEMPLATE_DIR} ] || \
		svn checkout ${CTEMPLATE_URL} -r ${CTEMPLATE_VERSION} ${CTEMPLATE_DIR}
fi
[ -f ${CTEMPLATE_LIB} ] || bash -c "cd ${CTEMPLATE_DIR} && \
	./configure --disable-shared \
	${CONFIGURE_FLAGS} --prefix=`readlink -f ${CTEMPLATE_PREFIX}` \
	&& make && make install"

# get and build gflags
GFLAGS_VERSION=2.0
GFLAGS_TAR=${EXTERNALS}/gflags-${GFLAGS_VERSION}.tar.gz
GFLAGS_DIR=${COMPILED}/gflags-${GFLAGS_VERSION}
GFLAGS_PREFIX=${GFLAGS_DIR}-prefix
GFLAGS_LIB=${GFLAGS_PREFIX}/lib/libgflags.a
GFLAGS_URL=https://gflags.googlecode.com/files/gflags-${GFLAGS_VERSION}.tar.gz
[ -f ${GFLAGS_TAR} ] || wget ${GFLAGS_URL} -O ${GFLAGS_TAR}
[ -d ${GFLAGS_DIR} ] || ( mkdir ${GFLAGS_DIR} && tar \
  --strip-components=1 -C ${GFLAGS_DIR} -xf ${GFLAGS_TAR} )
[ -f ${GFLAGS_LIB} ] || bash -c "cd ${GFLAGS_DIR} && ./configure \
  ${CONFIGURE_FLAGS} --prefix=`readlink -f ${GFLAGS_PREFIX}` \
  && make && make install"

# get and build libusb
LIBUSB_VERSION=1.0.9
LIBUSB_APIVERSION=1.0
LIBUSB_TAR=${EXTERNALS}/libusb-${LIBUSB_VERSION}.tar.bz2
LIBUSB_DIR=${COMPILED}/libusb-${LIBUSB_VERSION}
LIBUSB_PREFIX=${LIBUSB_DIR}-prefix
LIBUSB_LIB=${LIBUSB_PREFIX}/lib/libusb-${LIBUSB_APIVERSION}.a
LIBUSB_URL=http://sourceforge.net/projects/libusb/files/libusb-${LIBUSB_APIVERSION}/libusb-${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.tar.bz2
[ -f ${LIBUSB_TAR} ] || wget ${LIBUSB_URL} -O ${LIBUSB_TAR}
[ -d ${LIBUSB_DIR} ] || ( mkdir ${LIBUSB_DIR} && tar \
  --strip-components=1 -C ${LIBUSB_DIR} -xf ${LIBUSB_TAR} )
[ -f ${LIBUSB_LIB} ] || bash -c "cd ${LIBUSB_DIR} && ./configure \
	${CONFIGURE_FLAGS} --prefix=`readlink -f ${LIBUSB_PREFIX}` \
	&& make && make install"

# get the LLVM Compiler-RT source
COMPILER_RT_TAG=RELEASE_32/final
COMPILER_RT_VERSION=`echo ${COMPILER_RT_TAG} | sed s:/:_:`
COMPILER_RT_DIR=${EXTERNALS}/compiler-rt-${COMPILER_RT_VERSION}
COMPILER_RT_URL=http://llvm.org/svn/llvm-project/compiler-rt/tags/${COMPILER_RT_TAG}
[ -d ${COMPILER_RT_DIR} ] || svn checkout ${COMPILER_RT_URL} ${COMPILER_RT_DIR}

# get and build libevent
LIBEVENT_VERSION=2.0.21
LIBEVENT_TAR=${EXTERNALS}/libevent-${LIBEVENT_VERSION}.tar.gz
LIBEVENT_DIR=${COMPILED}/libevent-${LIBEVENT_VERSION}
LIBEVENT_PREFIX=${LIBEVENT_DIR}-prefix
LIBEVENT_LIB=${LIBEVENT_PREFIX}/lib/libevent.a
LIBEVENT_URL=https://github.com/downloads/libevent/libevent
LIBEVENT_URL=${LIBEVENT_URL}/libevent-${LIBEVENT_VERSION}-stable.tar.gz
[ -f ${LIBEVENT_TAR} ] || wget ${LIBEVENT_URL} -O ${LIBEVENT_TAR}
[ -d ${LIBEVENT_DIR} ] || ( mkdir ${LIBEVENT_DIR} && tar \
  --strip-components=1 -C ${LIBEVENT_DIR} -xf ${LIBEVENT_TAR} )
[ -f ${LIBEVENT_LIB} ] || bash -c "cd ${LIBEVENT_DIR} && ./configure \
	${CONFIGURE_FLAGS} --prefix=`readlink -f ${LIBEVENT_PREFIX}` \
	&& make && make install"

# get and build gmp
GMP_VERSION=5.1.3
GMP_TAR=${EXTERNALS}/gmp-${GMP_VERSION}.tar.lz
GMP_DIR=${COMPILED}/gmp-${GMP_VERSION}
GMP_PREFIX=${GMP_DIR}-prefix
GMP_LIB=${GMP_PREFIX}/lib/libgmp.a
GMP_URL=ftp://ftp.gmplib.org/pub/gmp/gmp-${GMP_VERSION}.tar.lz
[ -f ${GMP_TAR} ] || wget ${GMP_URL} -O ${GMP_TAR}
[ -d ${GMP_DIR} ] || ( mkdir ${GMP_DIR} && tar \
	--strip-components=1 -C ${GMP_DIR} -xf ${GMP_TAR} )
[ -f ${GMP_LIB} ] || bash -c "cd ${GMP_DIR} && ./configure \
	${CONFIGURE_FLAGS} --prefix=$(readlink -f ${GMP_PREFIX}) \
	&& make && make install"

# get and build libcdd
LIBCDD_VERSION=094g
LIBCDD_TAR=${EXTERNALS}/libcdd-${LIBCDD_VERSION}.tar.gz
LIBCDD_DIR=${COMPILED}/libcdd-${LIBCDD_VERSION}
LIBCDD_PREFIX=${LIBCDD_DIR}-prefix
LIBCDD_LIB=${LIBCDD_PREFIX}/lib/libcdd.a
LIBCDD_URL=ftp://ftp.ifor.math.ethz.ch/pub/fukuda/cdd/cddlib-${LIBCDD_VERSION}.tar.gz
[ -f ${LIBCDD_TAR} ] || \
        wget ${LIBCDD_URL} -O ${LIBCDD_TAR}
[ -d ${LIBCDD_DIR} ] || ( mkdir ${LIBCDD_DIR} && tar \
        --strip-components=1 -C ${LIBCDD_DIR} -xf ${LIBCDD_TAR} )
[ -f ${LIBCDD_LIB} ] || LDFLAGS=-L${GMP_PREFIX}/lib \
	bash -c "cd ${LIBCDD_DIR} && ./configure \
	--disable-shared ${CONFIGURE_FLAGS} \
	--prefix=$(readlink -f ${LIBCDD_PREFIX}) \
	&& make gmpdir=${GMP_PREFIX} && make install"

rm -rf ${TMPDIR}
