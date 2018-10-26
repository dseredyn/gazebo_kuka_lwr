cmake_minimum_required(VERSION 2.8.3)
project(gazebo_kuka_lwr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  geometry_msgs
#  visualization_msgs
#  urdf
  cmake_modules
)

find_package(GAZEBO REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(kdl_parser REQUIRED)

# Export package information (replaces catkin_package() macro)
catkin_package(
  LIBRARIES kuka_lwr_plugin
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(kuka_lwr_plugin src/kuka_lwr_plugin.cpp)
target_link_libraries(kuka_lwr_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS kuka_lwr_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

