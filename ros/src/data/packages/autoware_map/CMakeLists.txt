cmake_minimum_required(VERSION 2.8.3)
project(autoware_map)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  tf
  geometry_msgs
  visualization_msgs
  autoware_map_msgs
  vector_map
  vector_map_msgs
  op_planner
)

set(CMAKE_CXX_FLAGS " -o0 -g -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES autoware_map
 CATKIN_DEPENDS roscpp tf visualization_msgs geometry_msgs autoware_map_msgs vector_map vector_map_msgs op_planner
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)


add_library( autoware_map lib/autoware_map/autoware_map.cpp 
							lib/autoware_map/util.cpp lib/autoware_map/visualization.cpp
                			lib/autoware_map/opendrive_loader.cpp
                			lib/autoware_map/map_writer.cpp
                			lib/autoware_map/xml_helpers.cpp
                			lib/autoware_map/odr_spiral.cpp
                			lib/autoware_map/opendrive_road.cpp
                			lib/autoware_map/opendrive_objects.cpp)
add_dependencies( autoware_map ${catkin_EXPORTED_TARGETS} )
target_link_libraries( autoware_map ${catkin_LIBRARIES} )

add_executable( autoware2vectormap_converter
                nodes/autoware2vectormap_converter/autoware2vectormap_converter.cpp                
                nodes/autoware2vectormap_converter/converter_core.cpp
                nodes/autoware2vectormap_converter/vmap_visualization.cpp
                nodes/autoware2vectormap_converter/converter_util.cpp)
                
add_dependencies( autoware2vectormap_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries( autoware2vectormap_converter ${catkin_LIBRARIES} autoware_map)

add_executable( opendrive2autoware_converter
                nodes/opendrive2autoware_converter/opendrive2autoware_converter.cpp )
                
add_dependencies( opendrive2autoware_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries( opendrive2autoware_converter ${catkin_LIBRARIES} autoware_map)

## Install executables and/or libraries
install(TARGETS autoware_map autoware2vectormap_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
