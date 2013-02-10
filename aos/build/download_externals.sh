#!/bin/bash -e

AOS=`dirname $0`/..
EXTERNALS=${AOS}/externals

# get gccdist
GCCDIST=${EXTERNALS}/gccdist
[ -f ${GCCDIST}.zip ] || wget ftp://ftp.ni.com/pub/devzone/tut/updated_vxworks63gccdist.zip -O ${GCCDIST}.zip
[ -d ${GCCDIST} ] || ( cd ${EXTERNALS} && unzip -q ${GCCDIST}.zip )

# get eigen
EIGEN_VERSION=3.0.5
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
GTEST_DIR=${EXTERNALS}/gtest-${GTEST_VERSION}-p1
GTEST_ZIP=${EXTERNALS}/gtest-${GTEST_VERSION}.zip
TMPDIR=/tmp/$$-aos-tmpdir
[ -f ${GTEST_ZIP} ] || wget http://googletest.googlecode.com/files/gtest-${GTEST_VERSION}.zip -O ${GTEST_ZIP}
[ -d ${GTEST_DIR} ] || ( unzip ${GTEST_ZIP} -d ${TMPDIR} && mv ${TMPDIR}/gtest-${GTEST_VERSION} ${GTEST_DIR} && cd ${GTEST_DIR} && patch -p1 < ../gtest.patch )

# get and build ctemplate
CTEMPLATE_VERSION=2.2
CTEMPLATE_DIR=${EXTERNALS}/ctemplate-${CTEMPLATE_VERSION}
CTEMPLATE_PREFIX=${CTEMPLATE_DIR}-prefix
CTEMPLATE_LIB=${CTEMPLATE_PREFIX}/lib/libctemplate.a
CTEMPLATE_URL=http://ctemplate.googlecode.com/files
CTEMPLATE_URL=${CTEMPLATE_URL}/ctemplate-${CTEMPLATE_VERSION}.tar.gz
[ -f ${CTEMPLATE_DIR}.tar.gz ] || \
	wget ${CTEMPLATE_URL} -O ${CTEMPLATE_DIR}.tar.gz
[ -d ${CTEMPLATE_DIR} ] || ( mkdir ${CTEMPLATE_DIR} && tar \
	--strip-components=1 -C ${CTEMPLATE_DIR} -xf ${CTEMPLATE_DIR}.tar.gz )
[ -f ${CTEMPLATE_LIB} ] || env -i PATH="${PATH}" \
	CFLAGS='-m32' CXXFLAGS='-m32' LDFLAGS='-m32' \
	bash -c "cd ${CTEMPLATE_DIR} && ./configure --disable-shared \
	--prefix=`readlink -f ${CTEMPLATE_PREFIX}` && make && make install"
