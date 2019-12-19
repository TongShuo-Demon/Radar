set(daHua_shared_libs2
        GCBase_gcc421_v3_0
        GenApi_gcc421_v3_0
        Log_gcc421_v3_0
        MathParser_gcc421_v3_0
        log4cpp_gcc421_v3_0
        NodeMapData_gcc421_v3_0
        XmlParser_gcc421_v3_0
        ImageConvert
        MVSDK
        log4cpp
        )

add_executable(cam_node2
        ${PROJECT_SOURCE_DIR}/camera2/src/cam_mode.cpp
        ${PROJECT_SOURCE_DIR}/camera2/src/camParaConfig.cpp
        ${PROJECT_SOURCE_DIR}/camera2/src/shm.cpp
        ${PROJECT_SOURCE_DIR}/camera2/src/startFetch.cpp
        ${PROJECT_SOURCE_DIR}/camera2/src/streamRetrieve.cpp
        ${DIR_SRC})

target_link_libraries(cam_node2
                             ${daHua_shared_libs2}
                             ${OpenCV_LIBS}
                             rt -pthread
                             boost_system
                             boost_thread
                             timer)

