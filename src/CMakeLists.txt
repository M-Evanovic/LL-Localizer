add_library(${PROJECT_NAME}
        localizer.cc
        options.cc
        pointcloud_preprocess.cc
        utils.cc
        relocalizer/relocalizer.cpp
        relocalizer/gicp/fast_gicp.cpp 
        relocalizer/gicp/fast_gicp_st.cpp
        relocalizer/gicp/lsq_registration.cpp
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp livox_ros_driver_gencpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
