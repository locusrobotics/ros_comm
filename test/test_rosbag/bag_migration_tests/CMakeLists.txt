# This is going to need some work because the bag files on
# download.ros.org all have messages with the test_rosbag package
# name.  Of course the package name for this package now is rosbag.
# If we can have a way to pass an override package name into
# generate_messages() it might make it work, or we can migrate the bag
# files themselves.

set(MSG_DIRECTORY "msg_current" CACHE STRING "Messages directory")
message(STATUS "Building messages in: ${MSG_DIRECTORY}")

set(messages_common
  Converged.msg
  MigratedAddSub.msg
  MigratedExplicit.msg
  MigratedImplicit.msg
  MigratedMixed.msg
  PartiallyMigrated.msg
  Simple.msg
  SimpleMigrated.msg
  SubUnmigrated.msg
  Unmigrated.msg)

set(messages_msg_gen1
  Constants.msg
  Convergent.msg
  Renamed1.msg)

set(messages_msg_gen2
  Constants.msg
  Convergent.msg
  Renamed2.msg)

set(messages_msg_gen3
  Renamed3.msg)

set(messages_msg_gen4
  Constants.msg
  Renamed4.msg)

set(messages_msg_current
  Constants.msg
  Renamed5.msg)

add_message_files(
  DIRECTORY ${MSG_DIRECTORY}
  FILES ${messages_common} ${messages_${MSG_DIRECTORY}})

generate_messages(DEPENDENCIES rosgraph_msgs std_msgs)

# Copy test bag fixtures from the source tree into the build directory.
file(GLOB _migration_bags "${CMAKE_CURRENT_SOURCE_DIR}/test/*.bag")
file(COPY ${_migration_bags} DESTINATION "${PROJECT_BINARY_DIR}/test/")

configure_file(test/migrate_test.py.in
  ${PROJECT_BINARY_DIR}/test/migrate_test.py)
catkin_add_nosetests(${PROJECT_BINARY_DIR}/test/migrate_test.py)
configure_file(test/random_record.xml.in
  ${PROJECT_BINARY_DIR}/test/random_record.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_record.xml)
# Make sure that the random_record test runs before either of the random_play
# tests, both of which require a .bag file that the random_record test will
# write into /tmp. We're making an assumption about the target names generated
# by add_rostest(), but that naming scheme seems to be pretty stable.
configure_file(test/random_play.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play.xml
  DEPENDENCIES run_tests_test_rosbag_rostest_test_random_record.xml)
configure_file(test/random_play_sim.xml.in
  ${PROJECT_BINARY_DIR}/test/random_play_sim.xml)
add_rostest(${PROJECT_BINARY_DIR}/test/random_play_sim.xml
  DEPENDENCIES run_tests_test_rosbag_rostest_test_random_record.xml)

# Make sure that rostest random_record.xml is not executed twice, running in
# parallel (https://github.com/ros/ros_comm/pull/1651#issuecomment-482148146):
add_dependencies(run_tests_test_rosbag_rostest_test_random_play.xml run_tests_test_rosbag_rostest_test_random_play_sim.xml)
