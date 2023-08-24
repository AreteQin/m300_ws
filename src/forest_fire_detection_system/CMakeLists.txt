cmake_minimum_required(VERSION 2.8.3)
project(forest_fire_detection_system)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O0 -Wall -ggdb -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -fopenmp")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # Add elements to list

# ros package
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        roslib
        tf2
        tf2_ros
        image_transport
        dji_osdk_ros
        geometry_msgs
        sensor_msgs
        pcl_ros
        pcl_msgs
        pcl_conversions
        message_generation
        nav_msgs
        nmea_msgs
        std_msgs
        message_filters # for message_filter
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# yaml
find_library(YAML_CPP_LIBRARIES yaml-cpp)

# cv_bridge
set(cv_bridge_DIR ~/m300_ws/devel/share/cv_bridge/cmake)
find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})
link_directories(${cv_bridge_LIBRARY_DIRS})

# dji_osdk_ros
link_directories("~/m300_ws/devel/lib")
add_definitions(-DUSE_OPEN_CV_CONTRIB)
add_definitions(-DUSE_GPU)
add_definitions(-DADVANCED_SENSING)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV 4.5 
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
link_libraries(${GLOG_LIBRARIES})

# stereo_camera_vo
include_directories("ThirdParty/stereo_camera_vo/include")
link_directories("ThirdParty/stereo_camera_vo/lib")

add_message_files(
        FILES
        SingleFireFuse.msg
        SingleFireIR.msg
        SingleFireRGB.msg
)

add_service_files(
        FILES
        SegmentSmoke.srv
        ToggleGrabDataDepthEstimation.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        rospy
        roslib
        tf2
        tf2_ros
        std_msgs
        dji_osdk_ros
        geometry_msgs
        nav_msgs
        nmea_msgs
        message_runtime
)

set(VO_THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        pthread
        ${CSPARSE_LIBRARY}
        fmt::fmt
        stereo_camera_vo_common
        stereo_camera_vo_module
        stereo_camera_vo_tool
)

###########
## Build ##
###########

add_subdirectory(src)
