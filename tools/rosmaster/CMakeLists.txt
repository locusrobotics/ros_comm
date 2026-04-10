cmake_minimum_required(VERSION 3.0.2)
project(rosmaster)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  xmlrpcpp
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS xmlrpcpp
)

catkin_python_setup()

###########
## Build ##
###########

include_directories(
  include
)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/cpp/names.cpp
  src/cpp/thread_pool.cpp
  src/cpp/registrations.cpp
  src/cpp/param_server.cpp
  src/cpp/validators.cpp
  src/cpp/master_handler.cpp
  src/cpp/master.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  Threads::Threads
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_cpp)

add_executable(rosmaster_cpp src/cpp/main.cpp)
target_link_libraries(rosmaster_cpp
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  Threads::Threads
  yaml-cpp
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_lib rosmaster_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
