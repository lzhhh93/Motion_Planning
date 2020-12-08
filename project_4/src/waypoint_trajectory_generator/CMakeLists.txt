cmake_minimum_required(VERSION 2.8.3)
project(waypoint_trajectory_generator)

set(CMAKE_VERBOSE_MAKEFILE "false")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")
set(OOQP_INCLUDE_DIRS include)
#set(OOQP_LIB_DIRS /home/nubot229/github/test/lib)
#set(MA27_LIB /home/nubot229/github/test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  quadrotor_msgs
)
catkin_package()

find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(include
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS} 
        ${OOQP_INCLUDE_DIRS}
)

add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
        src/ooqp_inter.cpp
)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
    ooqpgensparse
    ooqpsparse
    ooqpgondzio
    ooqpbase
    ma27
    gfortran
    blas
)

add_executable(test_node
        src/test_node.cpp
        src/trajectory_generator_waypoint.cpp
        src/ooqp_inter.cpp
        ${OOQP_SRC_DIRS}
)

target_link_libraries(test_node
   ${catkin_LIBRARIES}
   ooqpgensparse
   ooqpsparse
   ooqpgondzio
   ooqpbase
   ma27
   gfortran
   blas
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OOQP_INCLUDE_DIRS}
)

#LINK_DIRECTORIES(OOQP_LIB_DIRS MA27_LIB)
add_executable(ooqp_test_node
        src/ooqp_test_node.cpp
        src/trajectory_generator_waypoint.cpp
        src/ooqp_inter.cpp
        ${OOQP_SRC_DIRS}
)
target_link_libraries(ooqp_test_node
    ${catkin_LIBRARIES}
    ooqpgensparse
    ooqpsparse
    ooqpgondzio
    ooqpbase
    ma27
    gfortran
    blas
)
