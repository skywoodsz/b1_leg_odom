cmake_minimum_required(VERSION 3.0.2)
project(b1_high_level)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# release
SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  realtime_tools
  tf2_ros
  sensor_msgs
  cheetah_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES aliengo_control
        CATKIN_DEPENDS roscpp 
        DEPENDS
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
link_directories(lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)


add_library(b1_control
        src/b1_control.cpp
        )
target_link_libraries(b1_control
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )

###########
##  EXE  ##
###########

add_executable(b1_control_node
        src/b1_control_node.cpp
)
target_link_libraries(b1_control_node
        b1_control
)






