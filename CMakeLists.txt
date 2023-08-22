cmake_minimum_required(VERSION 3.0.2)
project(robin_behavior_tree)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
  robin_bridge_generated
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robin_behavior_tree
  CATKIN_DEPENDS
    roscpp
    behaviortree_cpp_v3
    robin_bridge_generated
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robin_behavior_tree.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/behavior_tree_node.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME behavior_tree_node PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest_gtest(common_action_test test/common_action.test test/common_action_test.cpp)
  target_link_libraries(common_action_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
  add_rostest_gtest(semaphore_test test/semaphore.test test/semaphore_test.cpp)
  target_link_libraries(semaphore_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
  add_rostest_gtest(process_code_test test/process_code.test test/process_code_test.cpp)
  target_link_libraries(process_code_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

endif()