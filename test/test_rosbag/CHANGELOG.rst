^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.15.0 (2020-10-02)
-------------------
* Initializing the repeat_latched option (`#17 <https://github.com/locusrobotics/ros_comm/issues/17>`_)
  * Initializing the repeat_latched option and adding test
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/locusrobotics/ros_comm/issues/1792>`_)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* more Python 3 compatibility (`#1784 <https://github.com/locusrobotics/ros_comm/issues/1784>`_)
* Pickleable rosbag exceptions (`#1210 <https://github.com/locusrobotics/ros_comm/issues/1210>`_ revisited). (`#1652 <https://github.com/locusrobotics/ros_comm/issues/1652>`_)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* rostest: fix flaky hztests (`#1661 <https://github.com/locusrobotics/ros_comm/issues/1661>`_)
  * rostest: fix flaky hztests
  * add retry to all hztests
  * fix concerns
  * fix more wrong retry-attributes
* test_rosbag modernization: replaced BOOST_FOREACH with range-based for-loops (`#1642 <https://github.com/locusrobotics/ros_comm/issues/1642>`_)
* duplicate test nodes which aren't available to other packages, add missing dependencies (`#1611 <https://github.com/locusrobotics/ros_comm/issues/1611>`_)
* ROSBAG SNAPSHOT: address PR comments
* ROSBAG SNAPSHOT: documentation
* ROSBAG SNAPSHOT: move message definitions to new package rosbag_msgs
* ROSBAG SNAPSHOT: address TODOs
* ROSBAG SNAPSHOT: style formating
* ROSBAG SNAPSHOT: add prefix filename mode
* ROSBAG SNAPSHOT: improve testing
* ROSBAG SNAPSHOT: pull topic config from ROS params
* ROSBAG SNAPSHOT: clear on resume, write all topics, status bootstrap
* ROSBAG: add snapshot subcommand
* Contributors: Christopher Wecht, Dirk Thomas, Kevin Allen, Tom Moore, beetleskin
