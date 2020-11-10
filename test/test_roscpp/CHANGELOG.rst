^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2020-11-10)
-------------------

1.15.0 (2020-10-02)
-------------------
* `#1577 <https://github.com/locusrobotics/ros_comm/issues/1577>`_ revisited: Fix dynamic windowing for Topic Statistics (`#1695 <https://github.com/locusrobotics/ros_comm/issues/1695>`_)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Fixed test build errors. (`#1723 <https://github.com/locusrobotics/ros_comm/issues/1723>`_)
* Fixed issue occuring during alternating calls of getParamCached and setParam (`#1439 <https://github.com/locusrobotics/ros_comm/issues/1439>`_)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Added a check for missing dependencies and Docker container (`#1573 <https://github.com/locusrobotics/ros_comm/issues/1573>`_)
* Remove signals from find_package(Boost COMPONENTS ...) (`#1580 <https://github.com/locusrobotics/ros_comm/issues/1580>`_)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix test_roscpp build issues on Windows (`#1482 <https://github.com/locusrobotics/ros_comm/issues/1482>`_)
* reduce test threshold to avoid flakiness (`#1485 <https://github.com/locusrobotics/ros_comm/issues/1485>`_)
* Contributors: Christopher Wecht, Dirk Thomas, Johnson Shih, Maarten de Vries, Sean Yen, betab0t
