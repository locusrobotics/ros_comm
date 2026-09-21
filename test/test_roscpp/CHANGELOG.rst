^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* 1.15.6
* 1.15.5
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Fixed test build errors. (#1723)
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Added a check for missing dependencies and Docker container (#1573)
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix test_roscpp build issues on Windows (#1482)
* reduce test threshold to avoid flakiness (#1485)
* Contributors: Christopher Wecht, Dirk Thomas, Ivor Wanders, Jacob Perron, Johannes Meyer, Johnson Shih, Maarten de Vries, Mike Purvis, Sean Yen, Shane Loretz, betab0t

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* 1.15.6
* 1.15.5
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Fixed test build errors. (#1723)
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Added a check for missing dependencies and Docker container (#1573)
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix test_roscpp build issues on Windows (#1482)
* reduce test threshold to avoid flakiness (#1485)
* Contributors: Christopher Wecht, Dirk Thomas, Gary Servin, Ivor Wanders, Jacob Perron, Johannes Meyer, Johnson Shih, Maarten de Vries, Mike Purvis, Sean Yen, Shane Loretz, betab0t

1.19.0 (2023-09-25)
-------------------
* 1.18.0
* Update changelogs
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* 1.15.6
* 1.15.5
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Fixed test build errors. (#1723)
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Added a check for missing dependencies and Docker container (#1573)
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix test_roscpp build issues on Windows (#1482)
* reduce test threshold to avoid flakiness (#1485)
* Contributors: Christopher Wecht, Dirk Thomas, Gary Servin, Ivor Wanders, Jacob Perron, Johannes Meyer, Johnson Shih, Maarten de Vries, Mike Purvis, Sean Yen, Shane Loretz, betab0t

Forthcoming
-----------
* RST-13777 Fixing ros_comm tests (#54)
  * Fixing ros_comm tests
* Add log4cxx updates to ros_comm (#50)
  * Support for new log4cxx support
  * Added support for the changes to log4cxx version 1.1 that is in Noble.
* Fix "roscpp multithreaded spinners eat up CPU when callbacks take too long" (#2377)
  * * Fix a busy-wait loop in subscription queue.
  * Better fix for slow callbacks CPU throttling.
  * More versatile callback queue test.
  * Finished cherry-pick merge to melodic-devel.
  * libros: moved define
  * roscpp: implementet #1608 without ABI/API breaks
  * /test_roscpp: fake_message is in a header now...
  * test_roscpp: fixed sign-compare warning
  * stabilized test
  * CallbackQueue: use SteadyTime instead of WallTime to get independent of system-time changes
  * style only
  * Update clients/roscpp/include/ros/callback_queue.h
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
  Co-authored-by: Martin Pecka <peci1@seznam.cz>
  Co-authored-by: CTU base <robot@ctu-base>
  Co-authored-by: Martin Pecka <peckama2@fel.cvut.cz>
  Co-authored-by: Christopher Wecht <christopher.wechtstudent.kit.edu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
  * Fix subscription busy wait melodic (#2014)
  * roscpp: simplify implementation of CallbackQueue::callOne()
  Replace duplicate wait_for() call in CallbackQueue::callOne() with a loop and a call to wait_until().
  * roscpp: return TryAgain from CallbackQueue::callOne(timeout) immediately if timeout.isZero()
  ... and if none of the other result conditions holds (i.e. the queue is Empty or Disabled).
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
  ---------
  Co-authored-by: Christopher Wecht <cwecht@users.noreply.github.com>
  Co-authored-by: Martin Pecka <peci1@seznam.cz>
  Co-authored-by: CTU base <robot@ctu-base>
  Co-authored-by: Martin Pecka <peckama2@fel.cvut.cz>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
  (cherry picked from commit 114a14228c2c8afda8dafd320f4b25e5a770e69d)
* Fix occasional crash during shutdown when explicitly calling ros::start but not ros::shutdown (#2355)
  * Fix occasional crash during shutdown
  * add link to PR
  * comment
  * fix implementation
  * add missing hasError = true;
  * also call deInit
  * only deInit once
  * only deinit once
  * yet more fixes
  * add another test for init only
  * revert
  * preserve legacy behavior
  * add gtest wrapper
  * minimize code changes
  * add test
  * reduce changes even more
  * add comment
  * comment
  (cherry picked from commit 845f74602c7464e08ef5ac6fd9e26c97d0fe42c9)
* 1.17.0
* Update: #include <boost/bind.hpp> -> <boost/bind/bind.hpp> for boost 1.73 (#2348)
  * Update: #include <boost/bind.hpp> -> <boost/bind/bind.hpp> for boost 1.73
  Since boost 1.73, i.e. on Ubuntu 22.04, the old header issues a deprecation warning:
  ```
  /usr/include/boost/bind.hpp:36:1: note: ‘#pragma message:
  The practice of declaring the Bind placeholders (_1, _2, ...) in the global namespace is deprecated.
  Please use <boost/bind/bind.hpp> + using namespace boost::placeholders,
  or define BOOST_BIND_GLOBAL_PLACEHOLDERS to retain the current behavior.’
  ```
  While the new header <boost/bind/bind.hpp> is available for a long time (on 18.04 already)
  and the source has been updated to use boost::placeholders in #2023, the header was not yet updated.
  * For backwards compatibility: pull placeholders into global namespace
* Added init_options::NoSimTime to forcefully disable subscribing to /clock (#2342)
  * Added init_options::NoSimTime to forcefully disable subscribing to /clock.
  * Fix enum value
  * Added test for init_options::NoSimTime
  * Fixed test
* Fix occasional crash during shutdown when explicitly calling ros::start but not ros::shutdown (#2355)
  * Fix occasional crash during shutdown
  * add link to PR
  * comment
  * fix implementation
  * add missing hasError = true;
  * also call deInit
  * only deInit once
  * only deinit once
  * yet more fixes
  * add another test for init only
  * revert
  * preserve legacy behavior
  * add gtest wrapper
  * minimize code changes
  * add test
  * reduce changes even more
  * add comment
  * comment
* 1.16.0
* 1.15.15
* Move @jacobperron from maintainer to author (#2302)
* 1.15.14
* 1.15.13
* 1.15.12
* Contributors: David Gossow, Jacob Perron, Johannes Meyer, Kalpesh Lad, Martin Pecka, Michael Carroll, Robert Haschke, Shane Loretz, Tom Moore

1.23.0 (2025-02-04)
-------------------

1.22.0 (2024-09-16)
-------------------

1.21.0 (2024-06-17)
-------------------

1.20.0 (2024-02-02)
-------------------
* 1.19.0
* Update changelogs
* 1.18.0
* Update changelogs
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* 1.15.6
* 1.15.5
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Fixed test build errors. (#1723)
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Added a check for missing dependencies and Docker container (#1573)
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix test_roscpp build issues on Windows (#1482)
* reduce test threshold to avoid flakiness (#1485)
* Contributors: Christopher Wecht, Dirk Thomas, Gary Servin, Ivor Wanders, Jacob Perron, Johannes Meyer, Johnson Shih, Maarten de Vries, Mike Purvis, Sean Yen, Shane Loretz, betab0t

1.9.0 (2022-02-23)
-------------------
