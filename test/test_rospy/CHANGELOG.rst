^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rospy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.16.0 (2020-11-10)
-------------------

1.15.0 (2020-10-02)
-------------------
* more Python 3 compatibility (`#1796 <https://github.com/locusrobotics/ros_comm/issues/1796>`_)
  * keep subscribers around
  * more subprocess decode
  * add another seek(0), remove an unncessary one
* more Python 3 compatibility (`#1795 <https://github.com/locusrobotics/ros_comm/issues/1795>`_)
  * avoid using nose.tools without dependency being declared
  * seek(0)
  * subprocess decode
  * import urlparse
  * fix hash arg encode
  * print function
  * replace tabs used for indenting Python code with spaces
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/locusrobotics/ros_comm/issues/1792>`_)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* `#1577 <https://github.com/locusrobotics/ros_comm/issues/1577>`_ revisited: Fix dynamic windowing for Topic Statistics (`#1695 <https://github.com/locusrobotics/ros_comm/issues/1695>`_)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* more Python 3 compatibility (`#1785 <https://github.com/locusrobotics/ros_comm/issues/1785>`_)
* more Python 3 compatibility (`#1784 <https://github.com/locusrobotics/ros_comm/issues/1784>`_)
* Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message. (`#1703 <https://github.com/locusrobotics/ros_comm/issues/1703>`_)
  * Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message.
  Fixes https://github.com/ros/ros_comm/issues/1658.
  * spelling
* added is_legal_remap() to rosgraph to make remap-detection more precise (`#1683 <https://github.com/locusrobotics/ros_comm/issues/1683>`_)
  * added is_legal_remap() to rosgraph
  * test_rospy/test_rospy_client.py: fixed failing test
  * removed unrelated change
* more Python 3 compatibility (`#1783 <https://github.com/locusrobotics/ros_comm/issues/1783>`_)
* more Python 3 compatibility (`#1782 <https://github.com/locusrobotics/ros_comm/issues/1782>`_)
* Fixed issue occuring during alternating calls of getParamCached and setParam (`#1439 <https://github.com/locusrobotics/ros_comm/issues/1439>`_)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* test_rospy: added queue_size arguments to Publishers to avoid warning… (`#1643 <https://github.com/locusrobotics/ros_comm/issues/1643>`_)
  * test_rospy: added queue_size arguments to Publishers to avoid warnings II
  * a few forgotten Publisher
  * test_rospy: fixed test
* test_rospy: added queue_size arguments to Publishers to avoid warnings (`#1615 <https://github.com/locusrobotics/ros_comm/issues/1615>`_)
  * /test_rospy/talker: set publishers queue_size to supress warning
  * test_rospy: added missing queue_size paramters in Publishers to avoid warnings
* fix paths (and regex for paths) comparison issues (`#1592 <https://github.com/locusrobotics/ros_comm/issues/1592>`_)
  * add os.path.normcase for return value of Logger.FindCaller, escape windows path delimiter for regex comparison
  * escape os path separator for regular expression comparison
  * use os.path.sep to indicate path separator
  * use re.escape to escape metacharacters
  * apply escaping only when the string is used for comparison
* duplicate test nodes which aren't available to other packages, add missing dependencies (`#1611 <https://github.com/locusrobotics/ros_comm/issues/1611>`_)
* Contributors: Christopher Wecht, Dirk Thomas, James Xu, Martin Pecka
