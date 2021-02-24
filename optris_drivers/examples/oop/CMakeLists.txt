CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

PROJECT(IRIMAGEROOPEXAMPLE)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0")
SET(CMAKE_CXX_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0")

INCLUDE_DIRECTORIES(/usr/include/libirimager ../opengl)
LINK_DIRECTORIES(/usr/lib)

LINK_LIBRARIES(irdirectsdk udev GL GLU glut pthread usb-1.0)

ADD_EXECUTABLE(irimager_show_oop ../opengl/Obvious2D.cpp IRImagerGUI.cpp IRImagerHandler.cpp irimagerShowOOP.cpp)
