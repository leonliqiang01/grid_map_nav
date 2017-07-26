cmake_minimum_required(VERSION 2.8.3)
project(grid_map_nav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o2 -ffloat-store")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} geos-config --cflags")
set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} geos-config --ldflags")

find_package(catkin REQUIRED
	roscpp
	rospy
	visualization_msgs
	grid_map_ros
	grid_map_cv
	grid_map_msgs
)
find_package(Eigen3 3.1.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visgraph
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
set(SOURCE_FILES 
	src/main.cpp
	src/oamap_core.cpp
	src/oa_planner.cpp
	src/astar.cpp
	src/grid_path.cpp
	src/quadratic_calculator.cpp
)
include_directories( ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  include
)

link_directories(
  ${EIGEN3_LIBRARY_DIRS}
)
add_executable(grid_map_nav ${SOURCE_FILES})
add_dependencies(grid_map_nav visgraph_generate_messages_cpp)
target_link_libraries(grid_map_nav ${catkin_LIBRARIES})
