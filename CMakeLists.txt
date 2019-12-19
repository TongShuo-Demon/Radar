cmake_minimum_required(VERSION 3.14)
project(Radar)

set(CMAKE_CXX_STANDARD 11)

#设置执行文件输出目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(INC_DIR     /opt/DahuaTech/MVViewer/include
        ${Boost_INCLUDE_DIRS})

set(LINK_DIR    /opt/DahuaTech/MVViewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVViewer/lib
        ${PROJECT_SOURCE_DIR}/lib/cubot)

#寻找opencv库,并且添加头文件
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#寻找boost库，并且添加头文件
find_package(Boost REQUIRED COMPONENTS system atomic)
include_directories(${Boost_INCLUDE_DIRS})

#寻找eigen库，并且添加头文件
find_package(Eigen3  REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} ${INC_DIR} ${EIGEN3_INCLUDE_DIR})

#添加动态库,#添加系统关联库
link_directories(${LINK_DIR} )

#头文件搜索目录
include_directories( ${PROJECT_SOURCE_DIR}/my_radar/inc ${PROJECT_SOURCE_DIR}/camera/inc
        ${PROJECT_SOURCE_DIR}/camera2/inc ${PROJECT_SOURCE_DIR}/my_radar)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

#添加子目录
add_subdirectory(${PROJECT_SOURCE_DIR}/camera/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/camera2/src)


add_executable(Radar main.cpp camera/src/shm.cpp my_radar/src/pretreatment.cpp my_radar/inc/pretreatment.hpp
        my_radar/src/radar_main.cpp my_radar/inc/radar_main.hpp my_radar/inc/calculate.hpp my_radar/src/calculate.cpp
        my_radar/src/video_write.cpp  my_radar/inc/video_write.hpp my_radar/switch_function.hpp)

target_link_libraries(Radar ${OpenCV_LIBS} ${Eigen3_LIBRARIES} ${Boost_LIBRARY}
        boost_thread boost_system rt -lpthread timer -lboost_date_time)
