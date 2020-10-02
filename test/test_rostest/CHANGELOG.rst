^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rostest
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2020-10-02)
-------------------
* fix various test problems (`#1601 <https://github.com/locusrobotics/ros_comm/issues/1601>`_)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* Contributors: Dirk Thomas
