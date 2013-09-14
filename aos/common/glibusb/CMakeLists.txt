cmake_minimum_required(VERSION 2.8)
project(glibusb)

enable_testing()

find_package(Boost REQUIRED COMPONENTS thread date_time)	# need >= 1.48
find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(GFLAGS REQUIRED libgflags)
pkg_check_modules(GLOG REQUIRED libglog)

add_definitions("-std=c++11 -Wall -Wextra")

set(GTEST_INCLUDEDIR "/usr/src/gtest")
set(GTEST_SRCS
  /usr/src/gtest/src/gtest_main.cc
  /usr/src/gtest/src/gtest-all.cc
)


include_directories(
  ${Boost_INCLUDE_DIRS}
  /usr/include/libusb-1.0
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GTEST_INCLUDEDIR}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${GFLAGS_LIBRARY_DIRS}
  ${GLOG_LIBRARY_DIRS}
)

set(LIBRARIES
  ${Boost_LIBRARIES}
  usb-1.0
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  pthread
)

set(HEADERS
  gbuffer.h
  ghexdump.h
  glibusb_device_internal.h
  glibusb_endpoint.h
  glibusb_endpoint_internal.h
  glibusb.h
  glibusb_internal.h
  glibusb_transfer.h
)

#
add_library(glibusb
  gbuffer.cc
  glibusb.cc
  glibusb_device.cc
  glibusb_endpoint.cc
  glibusb_internal.cc
  glibusb_transfer.cc
  ghexdump.cc
)
target_link_libraries(glibusb ${LIBRARIES})

#
add_executable(gbuffer_test
  testing/gbuffer_test.cc
  ${GTEST_SRCS}
)
target_link_libraries(gbuffer_test glibusb ${LIBRARIES})
add_test(gbuffer_test gbuffer_test)

#
add_executable(glibusb_test
  testing/glibusb_test.cc
  ${GTEST_SRCS}
)
target_link_libraries(glibusb_test glibusb ${LIBRARIES})
add_test(glibusb_test glibusb_test)

install(TARGETS glibusb 
  LIBRARY DESTINATION lib COMPONENT Runtime
  ARCHIVE DESTINATION lib COMPONENT Runtime
)
install(FILES ${HEADERS}
  DESTINATION /usr/include/glibusb
  COMPONENT Development)

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_VERSION "1.0.14")
set(CPACK_PACKAGE_CONTACT "charliehotel@google.com")
include(CPack)
