cmake_minimum_required(VERSION 3.0.2)
project(controller_pausing)

find_package(catkin REQUIRED COMPONENTS
  controller_manager_msgs
  roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(controller_pauser src/controller_pauser.cpp)
add_dependencies(controller_pauser ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_pauser ${catkin_LIBRARIES})

install(TARGETS controller_pauser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)