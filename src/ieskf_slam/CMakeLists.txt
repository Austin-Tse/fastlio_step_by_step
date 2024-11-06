cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

## Compile as C++11, supported in ROS Kinetic and newer  -O3是一个优化选项，告诉编译器优化我们的代码。
SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
# add_compile_options(-std=c++17)
##定义PROJ_DIR这个全局变量            -D表示在编译器层面已经存在
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# YAML CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
# TODO 为什么要使用YAML CPP

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)




#SET(IESKF_SLAM_FILE
#    src/ieskf_slam/modules/frontend/frontend.cpp
#)

SET(IESKF_SLAM_FILE
        src/ieskf_slam/modules/frontend/frontend.cpp
        src/ieskf_slam/modules/ieskf/ieskf.cpp
        src/ieskf_slam/modules/map/rect_map_manager.cpp
        src/ieskf_slam/modules/frontback_Propagate/frontback_propagate.cpp
)



SET(NOETIC_FRONTEND_WRAPPER_FILE
    src/wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.cpp
        include/ieskf_slam/modules/ieskf/ieskf.h
        include/ieskf_slam/modules/map/rect_map_manager.h
        include/ieskf_slam/math/math.h
        include/ieskf_slam/type/measure_group.h
        src/ieskf_slam/modules/map/rect_map_manager.cpp
        src/ieskf_slam/modules/ieskf/ieskf.cpp
        include/ieskf_slam/math/SO3.h
        include/ieskf_slam/modules/frontback_Propagate/frontback_propagate.h
        src/ieskf_slam/modules/frontback_Propagate/frontback_propagate.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ieskf_frontend_node src/app/ros_noetic/ieskf_noetic_running_node.cpp
${IESKF_SLAM_FILE}
${NOETIC_FRONTEND_WRAPPER_FILE}
)

target_link_libraries(ieskf_frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
${YAML_CPP_LIBRARIES}
)