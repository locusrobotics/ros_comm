^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag_storage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2020-11-10)
-------------------

1.15.0 (2020-10-02)
-------------------
* Check for fclose returning 0 in rosbag_storage (`#1750 <https://github.com/locusrobotics/ros_comm/issues/1750>`_)
  fix `#1743 <https://github.com/locusrobotics/ros_comm/issues/1743>`_
* fix windows build for rosbag_storage (`#1687 <https://github.com/locusrobotics/ros_comm/issues/1687>`_)
  * fix broken changes. (`#55 <https://github.com/locusrobotics/ros_comm/issues/55>`_)
  Fix broken changes.
  * add comment, include rosbag_storage first
  * remove unhelpful comment
* fix `#1474 <https://github.com/locusrobotics/ros_comm/issues/1474>`_ move bag encryption plugins into separate library (`#1499 <https://github.com/locusrobotics/ros_comm/issues/1499>`_)
  * fix `#1474 <https://github.com/locusrobotics/ros_comm/issues/1474>`_ move bag encryption plugins into separate library
  This setup is required by the class_loader, which is part of the pluginlib, see  http://wiki.ros.org/class_loader#Caution_of_Linking_Directly_Against_Plugin_Libraries
  * fix unit tests of aes_encryptor
* rosbag_storage modernization: replaced BOOST_FOREACH with range-based for loops, used algorithm, where appropriated (`#1640 <https://github.com/locusrobotics/ros_comm/issues/1640>`_)
* rosbag_storage: fixed dangeling if-else (`#1637 <https://github.com/locusrobotics/ros_comm/issues/1637>`_)
* fix infinite loop in rosbag buffer resize (`#1623 <https://github.com/locusrobotics/ros_comm/issues/1623>`_)
  * fix infinite loop in rosbag buffer resize
  * grow to max size
* update CMakeLists.txt in rosbag_storage (`#1618 <https://github.com/locusrobotics/ros_comm/issues/1618>`_)
  * binplace rosbag_storage.dll to CATKIN_GLOBAL_IN
  * Move rosbag_storage.dll to package lib destination
  * Export NoEncryptor from Dll and install encryptor_plugins.xml for Windows (`#19 <https://github.com/locusrobotics/ros_comm/issues/19>`_)
  * update library install destination (`#47 <https://github.com/locusrobotics/ros_comm/issues/47>`_)
  * Remove unnecessary ROSBAG_STORAGE_DECL (`#28 <https://github.com/locusrobotics/ros_comm/issues/28>`_)
* fix various test problems (`#1601 <https://github.com/locusrobotics/ros_comm/issues/1601>`_)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* visibility macros update (`#1591 <https://github.com/locusrobotics/ros_comm/issues/1591>`_)
* Fix issues when built or run on Windows (`#1466 <https://github.com/locusrobotics/ros_comm/issues/1466>`_)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Christopher Wecht, Dirk Thomas, James Xu, Jeremie Deray, Johnson Shih, ipa-fez, jalkino

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------
* add in -D_FILE_OFFSET_BITS=64 on machines less than 64-bits (`#1406 <https://github.com/ros/ros_comm/issues/1406>`_)

1.14.0 (2018-05-21)
-------------------
* specialize BagCallbackT for MessageInstance (`#1374 <https://github.com/ros/ros_comm/issues/1374>`_)
* fix compiler warning of test_aes_encryptor (`#1376 <https://github.com/ros/ros_comm/issues/1376>`_)
* implement bag encryption/decryption (`#1206 <https://github.com/ros/ros_comm/issues/1206>`_)
* use boost::shared_ptr to fix memory leak (`#1373 <https://github.com/ros/ros_comm/issues/1373>`_)

1.13.6 (2018-02-05)
-------------------
* performance improvement for lower/upper bound (`#1223 <https://github.com/ros/ros_comm/issues/1223>`_)
* use namespaced logging macros of console_bridge instead of deprecated macros (`#1239 <https://github.com/ros/ros_comm/issues/1239>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* fix corrupted messages when reopening a rosbag with a different file (`#1176 <https://github.com/ros/ros_comm/issues/1176>`_)
* addd rosbag::Bag::isOpen (`#1190 <https://github.com/ros/ros_comm/issues/1190>`_)
* enable rosbag::Bag move operations if compiler support is available (`#1189 <https://github.com/ros/ros_comm/issues/1189>`_)
* check if bzfile\_ and lz4s\_ handle is valid before reading/writing/closing (`#1183 <https://github.com/ros/ros_comm/issues/1183>`_)
* fix an out of bounds read in rosbag::View::iterator::increment() (`#1191 <https://github.com/ros/ros_comm/issues/1191>`_)
* replace usage deprecated console_bridge macros (`#1149 <https://github.com/ros/ros_comm/issues/1149>`_)

1.13.2 (2017-08-15)
-------------------
* fix whitespace warnings with g++ 7 (`#1138 <https://github.com/ros/ros_comm/issues/1138>`_)
* remove deprecated dynamic exception specifications (`#1137 <https://github.com/ros/ros_comm/issues/1137>`_)

1.13.1 (2017-07-27)
-------------------
* fix buffer overflow vulnerability (`#1092 <https://github.com/ros/ros_comm/issues/1092>`_)
* fix rosbag::View::iterator copy assignment operator (`#1017 <https://github.com/ros/ros_comm/issues/1017>`_)
* fix open mode on Windows (`#1005 <https://github.com/ros/ros_comm/pull/1005>`_)
* add swap function instead of copy constructor / assignment operator for rosbag::Bag (`#1000 <https://github.com/ros/ros_comm/issues/1000>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* make Bag constructor explicit (`#835 <https://github.com/ros/ros_comm/pull/835>`_)

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------
* fix compiler warnings

1.11.17 (2016-03-11)
--------------------
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)

1.11.16 (2015-11-09)
--------------------

1.11.15 (2015-10-13)
--------------------

1.11.14 (2015-09-19)
--------------------

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* support large bagfiles (>2GB) on 32-bit systems (`#464 <https://github.com/ros/ros_comm/issues/464>`_)

1.11.10 (2014-12-22)
--------------------
* fix various defects reported by coverity

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------

1.11.5 (2014-06-24)
-------------------
* convert to use console bridge from upstream debian package (`ros/rosdistro#4633 <https://github.com/ros/rosdistro/issues/4633>`_)

1.11.4 (2014-06-16)
-------------------

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add lz4 compression to rosbag (Python and C++) (`#356 <https://github.com/ros/ros_comm/issues/356>`_)
* move rosbag dox to rosbag_storage (`#389 <https://github.com/ros/ros_comm/issues/389>`_)

1.11.0 (2014-03-04)
-------------------

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
