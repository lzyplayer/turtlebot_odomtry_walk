cmake_minimum_required(VERSION 2.8.3)
project(turtle_walk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp std_msgs  nav_msgs tf   geometry_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS} )

#show include
# message(STATUS "localization include_directories:" )
# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(STATUS "dir='${dir}'")
# endforeach()


###################################
## catkin specific configuration ##
###################################

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES  globalmap_provider_nodelet rt_localization_nodelet
 CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")
include_directories(
  include
  ${EIGEN_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
#    ${Driveworks_INCLUDE_DIRS}
#    ${CUDA_INCLUDE_DIRS}
)

#node

add_executable(walk src/walk.cpp)
add_dependencies(walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(walk ${catkin_LIBRARIES})


#install node
install(TARGETS walk
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
