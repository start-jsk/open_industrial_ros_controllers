# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(open_controllers_interface)

find_package(catkin REQUIRED COMPONENTS pr2_controller_manager roscpp std_srvs diagnostic_msgs diagnostic_updater)

catkin_package(
  DEPENDS
  CATKIN-DEPENDS pr2_controller_manager pr2_controller_manager roscpp std_srvs diagnostic_msgs diagnostic_updater
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/open_controllers_interface.cpp include/open_controllers_interface/open_controllers_interface.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)