cmake_minimum_required(VERSION 3.8)
project(lib_test)

# Add definitions
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "C:/local" CACHE PATH "Installation directory" FORCE)
endif()
if(MSVC)
    add_definitions(-D_WIN32_WINNT=0x0501)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs")
else()
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build")
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -Wunused-variable")
endif()

# Set dependent libraries
set(HEADER_DIR "" CACHE PATH "The path to dependent headers")
set(LIBRARY_DIR "" CACHE PATH "The path to dependent libraries")
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${HEADER_DIR}
    /usr/local/include
)
link_directories(
  	${LIBRARY_DIR}
    /usr/local/lib
)

#各ディレクトリ
set(SRC_DIR src)
set(INC_DIR lib)

#インクルードディレクトリ
include_directories(
)

#ソースコード
set(SRC 
    TestRFansLiDAR
    TestHeliosLiDAR
)

set(LIBS
  MiYALAB.Cpp
  MiYALAB.Cpp.Sensor.RFansLiDAR
  MiYALAB.Cpp.Device.LiDAR.RoboSense
  opencv_core
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_videoio
)
#ソースコードのコンパイル
foreach(target IN LISTS SRC)
    add_executable(${target} ${SRC_DIR}/${target}.cpp)
    target_link_libraries(${target} ${LIBS})
endforeach()
