^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rospy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.17.1 (2022-11-07)
-------------------

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* fix misspell. (#2066)
* 1.15.8
* 1.15.7
* 1.15.6
* 1.15.5
* Add rosout integration test (#1924)
  * add rosout integration test
  * update license to BSD
  * address PR comments
  * revert alphasorting of statements in CMakeLists.txt
* 1.15.4
* 1.15.3
* 1.15.2
* make statistics.test less flaky (#1899)
* 1.15.1
* fix flakyness of test_topic_statistics (#1896)
  * add assertion to narrow down error case
  * update test to ignore messages which are insufficient
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* fix lower bound of signed int (see ros/genpy#102) (#1781)
* more Python 3 compatibility (#1796)
  * keep subscribers around
  * more subprocess decode
  * add another seek(0), remove an unncessary one
* more Python 3 compatibility (#1795)
  * avoid using nose.tools without dependency being declared
  * seek(0)
  * subprocess decode
  * import urlparse
  * fix hash arg encode
  * print function
  * replace tabs used for indenting Python code with spaces
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* more Python 3 compatibility (#1785)
* more Python 3 compatibility (#1784)
* Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message. (#1703)
  * Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message.
  Fixes https://github.com/ros/ros_comm/issues/1658.
  * spelling
* added is_legal_remap() to rosgraph to make remap-detection more precise (#1683)
  * added is_legal_remap() to rosgraph
  * test_rospy/test_rospy_client.py: fixed failing test
  * removed unrelated change
* more Python 3 compatibility (#1783)
* more Python 3 compatibility (#1782)
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* test_rospy: added queue_size arguments to Publishers to avoid warning… (#1643)
  * test_rospy: added queue_size arguments to Publishers to avoid warnings II
  * a few forgotten Publisher
  * test_rospy: fixed test
* test_rospy: added queue_size arguments to Publishers to avoid warnings (#1615)
  * /test_rospy/talker: set publishers queue_size to supress warning
  * test_rospy: added missing queue_size paramters in Publishers to avoid warnings
* fix paths (and regex for paths) comparison issues (#1592)
  * add os.path.normcase for return value of Logger.FindCaller, escape windows path delimiter for regex comparison
  * escape os path separator for regular expression comparison
  * use os.path.sep to indicate path separator
  * use re.escape to escape metacharacters
  * apply escaping only when the string is used for comparison
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Christopher Wecht, Dirk Thomas, Jacob Perron, James Xu, Martin Pecka, Miaofei Mei, Shane Loretz, tomoya
