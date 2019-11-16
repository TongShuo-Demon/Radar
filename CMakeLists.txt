cmake_minimum_required(VERSION 3.14)
project(Radar)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV  REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} )

#头文件搜索目录
include_directories( ${PROJECT_SOURCE_DIR}/my_radar/inc)

#添加子目录

add_executable(Radar main.cpp my_radar/src/pretreatment.cpp my_radar/inc/pretreatment.hpp my_radar/src/radar_main.cpp my_radar/inc/radar_main.hpp my_radar/inc/calculate.hpp my_radar/src/calculate.cpp)

target_link_libraries(Radar ${OpenCV_LIBS} )
