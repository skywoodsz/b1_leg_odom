cmake_minimum_required(VERSION 3.0.2)
project(dog_estimator)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        realtime_tools
        cheetah_msgs
        tf2_ros
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/kf_estimator.cpp
        src/state_estimate.cpp
        src/terrain_estimator.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )


###########
##  EXE  ##
###########
add_executable(kf_estimator_node
        src/kf_estimator_node.cpp
        )
target_link_libraries(kf_estimator_node
        ${PROJECT_NAME}
        )

