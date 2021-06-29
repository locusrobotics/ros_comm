^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Passing time args through to the actual rosbag play executable (`#27 <https://github.com/locusrobotics/ros_comm/issues/27>`_)
* Writing out connection header for latched topics (`#26 <https://github.com/locusrobotics/ros_comm/issues/26>`_)
* Adding real clock time display option (`#25 <https://github.com/locusrobotics/ros_comm/issues/25>`_)
* Contributors: Tom Moore

1.16.0 (2020-11-10)
-------------------

1.15.0 (2020-10-02)
-------------------
* Initializing the repeat_latched option (`#17 <https://github.com/locusrobotics/ros_comm/issues/17>`_)
  * Initializing the repeat_latched option and adding test
* Fix latched timestamps (`#15 <https://github.com/locusrobotics/ros_comm/issues/15>`_)
  * Overwrite message receipt time to prevent gaps.
  * Revert unrelated whitespace changes
* Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
* Revert "Add pycryptodome as default on melodic (`#1609 <https://github.com/locusrobotics/ros_comm/issues/1609>`_)"
  This reverts commit ba7b5b13d9e1b82133bc18475c93c62bb0f3e85e.
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/locusrobotics/ros_comm/issues/1792>`_)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Add pycryptodome as default on melodic (`#1609 <https://github.com/locusrobotics/ros_comm/issues/1609>`_)
* Encrypted rosbag fixes for Python 3. (`#1777 <https://github.com/locusrobotics/ros_comm/issues/1777>`_)
* fixed `#1776 <https://github.com/locusrobotics/ros_comm/issues/1776>`_ deadcode (`#1786 <https://github.com/locusrobotics/ros_comm/issues/1786>`_)
  Signed-off-by: Daniel Wang <daniel.wang@canonical.com>
* rosbag fix: keep latched topics latched (`#1708 <https://github.com/locusrobotics/ros_comm/issues/1708>`_)
* Wrap the rosbag filter eval in a lambda (`#1712 <https://github.com/locusrobotics/ros_comm/issues/1712>`_)
* rosbag/record: fix signed int overflow (`#1741 <https://github.com/locusrobotics/ros_comm/issues/1741>`_)
* Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (`#1688 <https://github.com/locusrobotics/ros_comm/issues/1688>`_)
  * Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning
  * Change all usages of yaml.load to yaml.safe_load
  * Extend PyYAML's SafeLoader and use it with `yaml.load`
  Also added convenience functions for using this loader for reuse in
  `roslaunch`
  * fix typo in rosparam.yaml_load_all
  * Modify Loader and SafeLoader in yaml module directly
  * Revert whitespace change
  * Revert unrelated change to import through global variable construction
* Pickleable rosbag exceptions (`#1210 <https://github.com/locusrobotics/ros_comm/issues/1210>`_ revisited). (`#1652 <https://github.com/locusrobotics/ros_comm/issues/1652>`_)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* fix topic message count for rosbag indexed v1.2. (`#1648 <https://github.com/locusrobotics/ros_comm/issues/1648>`_)
* Fix wrong error handling in migration. (`#1639 <https://github.com/locusrobotics/ros_comm/issues/1639>`_)
  * Fix wrong error handling in migration.
  self.(old|new)_types is supposed to be a dictionary, not a list
  * Address PR 1639 comments.
  * remove duplicate warning message
* rosbag modernization: replaced BOOST_FOREACH with range-based for-loo… (`#1641 <https://github.com/locusrobotics/ros_comm/issues/1641>`_)
  * rosbag modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated
  * /rosbag: changed formatting
  * single level indentation
  * single level indentation
* fix IOError during Python file operation (`#1617 <https://github.com/locusrobotics/ros_comm/issues/1617>`_)
  * fix IOError from python file operation in r+ mode (`#41 <https://github.com/locusrobotics/ros_comm/issues/41>`_)
  * move file.seek into _stop_writing, add comment (`#42 <https://github.com/locusrobotics/ros_comm/issues/42>`_)
* add Windows.h usage explicitly (`#44 <https://github.com/locusrobotics/ros_comm/issues/44>`_) (`#1616 <https://github.com/locusrobotics/ros_comm/issues/1616>`_)
  * attempt to remove unused header.
  * Add windows.h usage explicitly
  * Update statistics.h
* [rosbag] Fix waitForSubscribers hanging with simtime (`#1543 <https://github.com/locusrobotics/ros_comm/issues/1543>`_)
  * [rosbag] Fix waitForSubscribers hanging with simtime
  Use Walltime for sleep as clock is not running yet
  * Simplify wallsleep in waitForSubscribers
  Co-Authored-By: AlexReimann <alexander.reimann@enway.ai>
* Publish last message from latch topics when start time > 0 (`#1537 <https://github.com/locusrobotics/ros_comm/issues/1537>`_)
  * Publish last message from latch topics when start time > 0
  * cuddle brace
  I know the ROS 1 style guide says otherwise but for my sanity of reading it I saved the vertical space
* Add a new option to publish when a bag write begin (`#1527 <https://github.com/locusrobotics/ros_comm/issues/1527>`_)
  * Add a new option to publish when a bag write begin
  * Add the option in rosbag_main to use rosbag record
* ROSBAG SNAPSHOT: address PR comments
* ROSBAG SNAPSHOT: documentation
* ROSBAG SNAPSHOT: clear topic buffer on backwards time
* ROSBAG SNAPSHOT: resolve filename path relative to client not server
* ROSBAG SNAPSHOT: move message definitions to new package rosbag_msgs
* ROSBAG SNAPSHOT: address TODOs
* ROSBAG SNAPSHOT: style formating
* ROSBAG SNAPSHOT: add prefix filename mode
* ROSBAG SNAPSHOT: improve testing
* ROSBAG SNAPSHOT: optional time range for trigger_snapshot service
* ROSBAG SNAPSHOT: cli arguments and client mode
* ROSBAG SNAPSHOT: bootstrap for client mode
* ROSBAG SNAPSHOT: pull topic config from ROS params
* ROSBAG SNAPSHOT: working status publishing
* ROSBAG SNAPSHOT: clear on resume, write all topics, status bootstrap
* ROSBAG: add snapshot subcommand
* Contributors: Alex Bencz, Alexander Reimann, AnthonyBirot, Christopher Wecht, Daniel Wang, Dirk Thomas, Enrique Fernández Perdomo, Eric Tappan, James Xu, Kevin Allen, Martijn Buijs, Maxime St-Pierre, Mike Purvis, Olivier Mangin, Tom Moore, kmiku7

1.14.3 (2018-08-06)
-------------------
* restore API compatibility (`#1473 <https://github.com/ros/ros_comm/issues/1473>`_) (regression from 1.14.0)
* throw BagException when disk is full (`#1451 <https://github.com/ros/ros_comm/issues/1451>`_)

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* keep connection header info in rosbag filter/compress (`#1372 <https://github.com/ros/ros_comm/issues/1372>`_)
* implement bag encryption/decryption (`#1206 <https://github.com/ros/ros_comm/issues/1206>`_)
* add TransportHint options --tcpnodelay and --udp (`#1295 <https://github.com/ros/ros_comm/issues/1295>`_)
* fix check for header first in rosbag play for rate control topic (`#1352 <https://github.com/ros/ros_comm/issues/1352>`_)

1.13.6 (2018-02-05)
-------------------
* return an error status on error in rosbag (`#1257 <https://github.com/ros/ros_comm/issues/1257>`_)
* fix warn of --max-splits without --split (`#1237 <https://github.com/ros/ros_comm/issues/1237>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* fix publishing of selected topics from bag file (`#1156 <https://github.com/ros/ros_comm/issues/1156>`_)
* fix Python 3 compatibility (`#1150 <https://github.com/ros/ros_comm/issues/1150>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* fix handling connections without indices (`#1109 <https://github.com/ros/ros_comm/pull/1109>`_)
* improve message of check command (`#1067 <https://github.com/ros/ros_comm/pull/1067>`_)
* fix BZip2 inclusion (`#1016 <https://github.com/ros/ros_comm/pull/1016>`_)
* expose rate-control-topic and rate-control-max-delay args to command line tool (`#1015 <https://github.com/ros/ros_comm/pull/1015>`_)
* improve migration rule generation (`#1009 <https://github.com/ros/ros_comm/pull/1009>`_, `#1010 <https://github.com/ros/ros_comm/pull/1010>`_, `#1011 <https://github.com/ros/ros_comm/pull/1011>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* throw exception instead of accessing invalid memory (`#971 <https://github.com/ros/ros_comm/pull/971>`_)
* move headers to include/xmlrpcpp (`#962 <https://github.com/ros/ros_comm/issues/962>`_)
* added option wait-for-subscriber to rosbag play (`#959 <https://github.com/ros/ros_comm/issues/959>`_)
* terminate underlying rosbag play, record  on SIGTERM (`#951 <https://github.com/ros/ros_comm/issues/951>`_)
* add pause service for rosbag player (`#949 <https://github.com/ros/ros_comm/issues/949>`_)
* add rate-control-topic and rate-control-max-delay. (`#947 <https://github.com/ros/ros_comm/issues/947>`_)

1.12.6 (2016-10-26)
-------------------
* fix BagMigrationException in migrate_raw (`#917 <https://github.com/ros/ros_comm/issues/917>`_)

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* set default values for min_space and min_space_str (`#883 <https://github.com/ros/ros_comm/issues/883>`_)
* record a maximum number of splits and then begin deleting old files (`#866 <https://github.com/ros/ros_comm/issues/866>`_)
* allow 64-bit sizes to be passed to robag max_size (`#865 <https://github.com/ros/ros_comm/issues/865>`_)
* update rosbag filter progress meter to use raw uncompressed input size (`#857 <https://github.com/ros/ros_comm/issues/857>`_)

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* promote the result of read_messages to a namedtuple (`#777 <https://github.com/ros/ros_comm/pull/777>`_)
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------
* add missing parameter to AdvertiseOptions::createAdvertiseOptions (`#733 <https://github.com/ros/ros_comm/issues/733>`_)

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)

1.11.16 (2015-11-09)
--------------------
* show size unit for --size of rosbag record in help string (`#697 <https://github.com/ros/ros_comm/pull/697>`_)

1.11.15 (2015-10-13)
--------------------
* add option --prefix for prefixing output topics (`#626 <https://github.com/ros/ros_comm/pull/626>`_)

1.11.14 (2015-09-19)
--------------------
* reduce memory usage by using slots for IndexEntry types (`#613 <https://github.com/ros/ros_comm/pull/613>`_)
* remove duplicate topics (`#647 <https://github.com/ros/ros_comm/issues/647>`_)
* better exception when calling get_start_time / get_end_time on empty bags (`#657 <https://github.com/ros/ros_comm/pull/657>`_)
* make support for lz4 in rosbag optional (`#642 <https://github.com/ros/ros_comm/pull/642>`_)
* fix handling of "play --topics" (`#620 <https://github.com/ros/ros_comm/issues/620>`_)

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* add support for pausing when specified topics are about to be published (`#569 <https://github.com/ros/ros_comm/pull/569>`_)

1.11.10 (2014-12-22)
--------------------
* add option to specify the minimum disk space at which recording is stopped (`#500 <https://github.com/ros/ros_comm/pull/500>`_)
* add convenience API to Python rosbag (`#508 <https://github.com/ros/ros_comm/issues/508>`_)
* fix delay on detecting a running rosmaster with use_sim_time set (`#532 <https://github.com/ros/ros_comm/pull/532>`_)

1.11.9 (2014-08-18)
-------------------

1.11.8 (2014-08-04)
-------------------

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------
* fix rosbag record prefix (`#449 <https://github.com/ros/ros_comm/issues/449>`_)

1.11.5 (2014-06-24)
-------------------
* Fix typo in rosbag usage

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#430 <https://github.com/ros/ros_comm/issues/430>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add lz4 compression to rosbag (Python and C++) (`#356 <https://github.com/ros/ros_comm/issues/356>`_)
* fix rosbag record --node (`#357 <https://github.com/ros/ros_comm/issues/357>`_)
* move rosbag dox to rosbag_storage (`#389 <https://github.com/ros/ros_comm/issues/389>`_)

1.11.0 (2014-03-04)
-------------------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------
* readd missing declaration of rosbag::createAdvertiseOptions (`#338 <https://github.com/ros/ros_comm/issues/338>`_)

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* fix return value on platforms where char is unsigned.
* fix usage of boost include directories

1.9.50 (2013-10-04)
-------------------
* add chunksize option to rosbag record

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* search for exported rosbag migration rules based on new package rosbag_migration_rule

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------
* fix crash in bag migration (`#239 <https://github.com/ros/ros_comm/issues/239>`_)

1.9.45 (2013-06-06)
-------------------
* added option '--duration' to 'rosbag play' (`#121 <https://github.com/ros/ros_comm/issues/121>`_)
* fix missing newlines in rosbag error messages (`#237 <https://github.com/ros/ros_comm/issues/237>`_)
* fix flushing for tools like 'rosbag compress' (`#237 <https://github.com/ros/ros_comm/issues/237>`_)

1.9.44 (2013-03-21)
-------------------
* fix various issues on Windows (`#189 <https://github.com/ros/ros_comm/issues/189>`_)

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* added option '--duration' to 'rosrun rosbag play' (`#121 <https://github.com/ros/ros_comm/issues/121>`_)
* add error message to rosbag when using same in and out file (`#171 <https://github.com/ros/ros_comm/issues/171>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* fix bagsort script (`#42 <https://github.com/ros/ros_comm/issues/42>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
