CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(IRIMAGERMINIMALEXAMPLE)

OPTION(USE_LIBUSB "use libusb instead of hidraw interface")

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0")
SET(CMAKE_CXX_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0")

SET(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(/usr/include/libirimager)
LINK_DIRECTORIES(/usr/lib)

LINK_LIBRARIES(irdirectsdk udev pthread)
#####################################################################################
# If your applications do not rely on hidraw, use usb-1.0 as additional dependency. #
# This needs for a library linked against usb-1.0. Take care to install the correct #
# Debian package.                                                                   #
#####################################################################################
IF(USE_LIBUSB)
LINK_LIBRARIES(usb-1.0)
ENDIF(USE_LIBUSB)

ADD_EXECUTABLE(ir_direct_daemon irDirectDaemon.cpp)
