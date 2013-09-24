#!/bin/bash

set -e

AOS=`dirname $0`/..
EXTERNALS=${AOS}/externals

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
LIBJPEG_DIR=${EXTERNALS}/jpeg-${LIBJPEG_VERSION}
# NOTE: this directory ends up in #include names
LIBJPEG_PREFIX=${EXTERNALS}/libjpeg
LIBJPEG_LIB=${LIBJPEG_PREFIX}/lib/libjpeg.a
LIBJPEG_TAR=${EXTERNALS}/jpegsrc.v${LIBJPEG_VERSION}.tar.gz
[ -f ${LIBJPEG_TAR} ] || wget http://www.ijg.org/files/jpegsrc.v${LIBJPEG_VERSION}.tar.gz -O ${LIBJPEG_TAR}
[ -d ${LIBJPEG_DIR} ] || ( mkdir ${LIBJPEG_DIR} && tar --strip-components=1 -C ${LIBJPEG_DIR} -xf ${LIBJPEG_TAR} )
[ -f ${LIBJPEG_LIB} ] || env -i PATH="${PATH}" bash -c "cd ${LIBJPEG_DIR} && CFLAGS='-m32' ./configure --disable-shared --prefix=`readlink -f ${LIBJPEG_PREFIX}` && make && make install"

# get gtest
GTEST_VERSION=1.6.0
GTEST_DIR=${EXTERNALS}/gtest-${GTEST_VERSION}-p2
GTEST_ZIP=${EXTERNALS}/gtest-${GTEST_VERSION}.zip
TMPDIR=/tmp/$$-aos-tmpdir
[ -f ${GTEST_ZIP} ] || wget http://googletest.googlecode.com/files/gtest-${GTEST_VERSION}.zip -O ${GTEST_ZIP}
[ -d ${GTEST_DIR} ] || ( unzip ${GTEST_ZIP} -d ${TMPDIR} && mv ${TMPDIR}/gtest-${GTEST_VERSION} ${GTEST_DIR} && cd ${GTEST_DIR} && patch -p1 < ../gtest.patch )

# get and build ctemplate
# This is the next revision after the 2.2 release and it only adds spaces to
# make gcc 4.7 with --std=c++11 happy (user-defined string literals...).
CTEMPLATE_VERSION=129
CTEMPLATE_DIR=${EXTERNALS}/ctemplate-${CTEMPLATE_VERSION}
CTEMPLATE_PREFIX=${CTEMPLATE_DIR}-prefix
CTEMPLATE_LIB=${CTEMPLATE_PREFIX}/lib/libctemplate.a
CTEMPLATE_URL=http://ctemplate.googlecode.com
if [[ "${CTEMPLATE_VERSION}" =~ /\./ ]]; then
	CTEMPLATE_URL=${CTEMPLATE_URL}/files/ctemplate-${CTEMPLATE_VERSION}.tar.gz
	[ -f ${CTEMPLATE_DIR}.tar.gz ] || \
		wget ${CTEMPLATE_URL} -O ${CTEMPLATE_DIR}.tar.gz
	[ -d ${CTEMPLATE_DIR} ] || ( mkdir ${CTEMPLATE_DIR} && tar \
		--strip-components=1 -C ${CTEMPLATE_DIR} -xf ${CTEMPLATE_DIR}.tar.gz )
else
	CTEMPLATE_URL=${CTEMPLATE_URL}/svn/trunk
	[ -d ${CTEMPLATE_DIR} ] || \
		svn checkout ${CTEMPLATE_URL} -r ${CTEMPLATE_VERSION} ${CTEMPLATE_DIR}
fi
[ -f ${CTEMPLATE_LIB} ] || env -i PATH="${PATH}" \
	CFLAGS='-m32' CXXFLAGS='-m32' LDFLAGS='-m32' \
	bash -c "cd ${CTEMPLATE_DIR} && ./configure --disable-shared \
	--prefix=`readlink -f ${CTEMPLATE_PREFIX}` && make && make install"

# get and build gflags
GFLAGS_VERSION=2.0
GFLAGS_DIR=${EXTERNALS}/gflags-${GFLAGS_VERSION}
GFLAGS_PREFIX=${GFLAGS_DIR}-prefix
GFLAGS_LIB=${GFLAGS_PREFIX}/lib/libgflags.a
GFLAGS_URL=https://gflags.googlecode.com/files/gflags-${GFLAGS_VERSION}.tar.gz
[ -f ${GFLAGS_DIR}.tar.gz ] || wget ${GFLAGS_URL} -O ${GFLAGS_DIR}.tar.gz
[ -d ${GFLAGS_DIR} ] || ( mkdir ${GFLAGS_DIR} && tar \
  --strip-components=1 -C ${GFLAGS_DIR} -xf ${GFLAGS_DIR}.tar.gz )
[ -f ${GFLAGS_LIB} ] || env -i PATH="${PATH}" \
  CFLAGS='-m32' CXXFLAGS='-m32' LDFLAGS='-m32' \
  bash -c "cd ${GFLAGS_DIR} && ./configure \
  --prefix=`readlink -f ${GFLAGS_PREFIX}` && make && make install"

# get and build libusb
LIBUSB_VERSION=1.0.9
LIBUSB_APIVERSION=1.0
LIBUSB_DIR=${EXTERNALS}/libusb-${LIBUSB_VERSION}
LIBUSB_PREFIX=${LIBUSB_DIR}-prefix
LIBUSB_LIB=${LIBUSB_PREFIX}/lib/libusb-${LIBUSB_APIVERSION}.a
LIBUSB_URL=http://sourceforge.net/projects/libusb/files/libusb-${LIBUSB_APIVERSION}/libusb-${LIBUSB_VERSION}/libusb-${LIBUSB_VERSION}.tar.bz2
[ -f ${LIBUSB_DIR}.tar.bz2 ] || wget ${LIBUSB_URL} -O ${LIBUSB_DIR}.tar.bz2
[ -d ${LIBUSB_DIR} ] || ( mkdir ${LIBUSB_DIR} && tar \
  --strip-components=1 -C ${LIBUSB_DIR} -xf ${LIBUSB_DIR}.tar.bz2 )
[ -f ${LIBUSB_LIB} ] || env -i PATH="${PATH}" \
  CFLAGS='-m32' CXXFLAGS='-m32' LDFLAGS='-m32' \
  bash -c "cd ${LIBUSB_DIR} && ./configure \
  --prefix=`readlink -f ${LIBUSB_PREFIX}` && make && make install"

# get the LLVM Compiler-RT source
COMPILER_RT_TAG=RELEASE_32/final
COMPILER_RT_VERSION=`echo ${COMPILER_RT_TAG} | sed s:/:_:`
COMPILER_RT_DIR=${EXTERNALS}/compiler-rt-${COMPILER_RT_VERSION}
COMPILER_RT_URL=http://llvm.org/svn/llvm-project/compiler-rt/tags/${COMPILER_RT_TAG}
[ -d ${COMPILER_RT_DIR} ] || svn checkout ${COMPILER_RT_URL} ${COMPILER_RT_DIR}

# get and build libevent
LIBEVENT_VERSION=2.0.21
LIBEVENT_DIR=${EXTERNALS}/libevent-${LIBEVENT_VERSION}
LIBEVENT_PREFIX=${LIBEVENT_DIR}-prefix
LIBEVENT_LIB=${LIBEVENT_PREFIX}/lib/libevent.a
LIBEVENT_URL=https://github.com/downloads/libevent/libevent
LIBEVENT_URL=${LIBEVENT_URL}/libevent-${LIBEVENT_VERSION}-stable.tar.gz
[ -f ${LIBEVENT_DIR}.tar.gz ] || wget ${LIBEVENT_URL} -O ${LIBEVENT_DIR}.tar.gz
[ -d ${LIBEVENT_DIR} ] || ( mkdir ${LIBEVENT_DIR} && tar \
  --strip-components=1 -C ${LIBEVENT_DIR} -xf ${LIBEVENT_DIR}.tar.gz )
[ -f ${LIBEVENT_LIB} ] || env -i PATH="${PATH}" \
  CFLAGS='-m32' CXXFLAGS='-m32' LDFLAGS='-m32' \
  bash -c "cd ${LIBEVENT_DIR} && ./configure \
  --prefix=`readlink -f ${LIBEVENT_PREFIX}` && make && make install"
