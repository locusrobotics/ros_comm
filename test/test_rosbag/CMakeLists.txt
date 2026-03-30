cmake_minimum_required(VERSION 3.0.2)

project(test_rosbag)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS message_generation rosbag rosconsole roscpp rosgraph_msgs rostest rosunit topic_tools xmlrpcpp)

if(CATKIN_ENABLE_TESTING)
  find_package(Boost REQUIRED COMPONENTS regex program_options)
  find_package(BZip2 REQUIRED)

  add_subdirectory(bag_migration_tests)
endif()

catkin_package()

if(CATKIN_ENABLE_TESTING)
  include_directories(${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

  # Copy test bag fixtures from the source tree into the build directory so all
  # tests can reference them via PROJECT_BINARY_DIR/test/ without network access.
  file(COPY
    test/test_indexed_1.2.bag
    test/chatter_50hz.bag
    test/test_future_version_2.1.bag
    test/test_rosbag_latched_pub.bag
    DESTINATION ${PROJECT_BINARY_DIR}/test/)

  catkin_add_nosetests(test/test_bag.py)

  configure_file(test/test_bag.cpp.in 
                 ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
  catkin_add_gtest(test_bag ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
  if(TARGET test_bag)
    target_link_libraries(test_bag ${catkin_LIBRARIES})
  endif()

  configure_file(test/play_play.test.in 
                 ${PROJECT_BINARY_DIR}/test/play_play.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/play_play.test)
  configure_file(test/rosbag_play.test.in 
                 ${PROJECT_BINARY_DIR}/test/rosbag_play.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/rosbag_play.test)

  add_rostest(test/latched_pub.test)
  configure_file(test/latched_sub.test.in ${PROJECT_BINARY_DIR}/test/latched_sub.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/latched_sub.test)
  add_rostest(test/record_two_publishers.test)
  add_rostest(test/record_one_publisher_two_topics.test)
  add_rostest(test/record_sigint_cleanup.test)
  add_rostest(test/record_sigterm_cleanup.test)

  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(double_pub EXCLUDE_FROM_ALL test/double_pub.cpp)
  target_link_libraries(double_pub ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
  add_dependencies(double_pub ${rosgraph_msgs_EXPORTED_TARGETS})
  if(TARGET tests)
    add_dependencies(tests double_pub)
  endif()
endif()
