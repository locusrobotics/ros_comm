^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roscpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.11 (2021-04-06)
--------------------

1.15.10 (2021-03-18)
--------------------
* Fix for deadlock issue related to timers (`#2121 <https://github.com/ros/ros_comm/issues/2121>`_)
* Fix getNumPublishers() to only count fully connected (`#2107 <https://github.com/ros/ros_comm/issues/2107>`_)
* Replace message assertion with logging in order to have release modes to fail in compilation when msg type mismatches occur (`#2096 <https://github.com/ros/ros_comm/issues/2096>`_)
* Contributors: C. Andy Martin, Ivor Wanders, Tahsincan Köse

1.15.9 (2020-10-16)
-------------------
* Fix deadlock when service connection is dropped (`#2074 <https://github.com/ros/ros_comm/issues/2074>`_)
* Update maintainers (`#2075 <https://github.com/ros/ros_comm/issues/2075>`_)
* Fix case where accessing cached parameters shuts down another node (`#2068 <https://github.com/ros/ros_comm/issues/2068>`_)
* Fix spelling (`#2066 <https://github.com/ros/ros_comm/issues/2066>`_)
* Fix Lost Wake Bug in ROSOutAppender (`#2033 <https://github.com/ros/ros_comm/issues/2033>`_)
* Fix compatibility issue with boost 1.73 and above (`#2023 <https://github.com/ros/ros_comm/issues/2023>`_)
* Contributors: Adel Fakih, Chen Lihui, Sean Yen, Shane Loretz, tomoya

1.15.8 (2020-07-23)
-------------------
* change is_async_connected to use epoll when available (`#1983 <https://github.com/ros/ros_comm/issues/1983>`_)
* allow mixing latched and unlatched publishers (`#1991 <https://github.com/ros/ros_comm/issues/1991>`_)

1.15.7 (2020-05-28)
-------------------
* fix Windows build break (`#1961 <https://github.com/ros/ros_comm/issues/1961>`_) (regression from 1.15.5)

1.15.6 (2020-05-21)
-------------------
* fix a bug that using a destroyed connection object (`#1950 <https://github.com/ros/ros_comm/issues/1950>`_)

1.15.5 (2020-05-15)
-------------------
* check if async socket connect is success or failure before TransportTCP::read() and TransportTCP::write() (`#1954 <https://github.com/ros/ros_comm/issues/1954>`_)
* fix bug that connection drop signal related funtion throw a bad_weak exception (`#1940 <https://github.com/ros/ros_comm/issues/1940>`_)
* multiple latched publishers per process on the same topic (`#1544 <https://github.com/ros/ros_comm/issues/1544>`_)
* fix negative numbers in ros statistics (`#1531 <https://github.com/ros/ros_comm/issues/1531>`_)
* remove extra \n in ROS_DEBUG (`#1925 <https://github.com/ros/ros_comm/issues/1925>`_)

1.15.4 (2020-03-19)
-------------------
* restrict boost dependencies to components used (`#1871 <https://github.com/ros/ros_comm/issues/1871>`_)

1.15.3 (2020-02-28)
-------------------
* remove Boost version check since Noetic only targets platforms with 1.67+ (`#1903 <https://github.com/ros/ros_comm/issues/1903>`_)

1.15.2 (2020-02-25)
-------------------
* export missing Boost dependency (`#1898 <https://github.com/ros/ros_comm/issues/1898>`_)

1.15.1 (2020-02-24)
-------------------
* fix missing boost dependencies (`#1895 <https://github.com/ros/ros_comm/issues/1895>`_)

1.15.0 (2020-02-21)
-------------------

1.14.4 (2020-02-20)
-------------------
* add default ROS_MASTER_URI (`#1666 <https://github.com/ros/ros_comm/issues/1666>`_)
* add default assignment operator for various classes (`#1888 <https://github.com/ros/ros_comm/issues/1888>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* do not display error message if poll yields EINTR (`#1868 <https://github.com/ros/ros_comm/issues/1868>`_)
* [windows] portable duration cast (`#1882 <https://github.com/ros/ros_comm/issues/1882>`_)
* drop custom implementation of boost::condition_variable to fix busy-wait spinning (`#1878 <https://github.com/ros/ros_comm/issues/1878>`_)
* disable rosout via ROSCPP_NO_ROSOUT environment variable (`#1858 <https://github.com/ros/ros_comm/issues/1858>`_)
* [windows] conditionally guard sys/socket.h (`#1876 <https://github.com/ros/ros_comm/issues/1876>`_)
* explicit include of socket.h to support FreeBSD (`#1864 <https://github.com/ros/ros_comm/issues/1864>`_)
* remove DEBUG statements from getImpl (`#1823 <https://github.com/ros/ros_comm/issues/1823>`_)
* use c++11 std::snprintf (`#1820 <https://github.com/ros/ros_comm/issues/1820>`_)
* TransportTCP: Allow socket() to return 0 (`#1707 <https://github.com/ros/ros_comm/issues/1707>`_)
* fix dynamic windowing for Topic Statistics (`#1695 <https://github.com/ros/ros_comm/issues/1695>`_)
* service_publication: removed int-bool-comparison (`#1710 <https://github.com/ros/ros_comm/issues/1710>`_)
* add Timer::isValid() const (`#1779 <https://github.com/ros/ros_comm/issues/1779>`_)
* add possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message (`#1703 <https://github.com/ros/ros_comm/issues/1703>`_)
* fix segfault in TransportPublisherLink (`#1714 <https://github.com/ros/ros_comm/issues/1714>`_)
* TransportTCP: enable poll event POLLRDHUP to detect dead connections properly (`#1704 <https://github.com/ros/ros_comm/issues/1704>`_)
* transportUDP: zero-initialize sockaddr_in object (`#1740 <https://github.com/ros/ros_comm/issues/1740>`_)
* unregisterService returns result of execute("unregisterService") (`#1751 <https://github.com/ros/ros_comm/issues/1751>`_)
* use safe string check (`#1771 <https://github.com/ros/ros_comm/issues/1771>`_)
* fix memory leak of global variable (`#1503 <https://github.com/ros/ros_comm/issues/1503>`_)
* fix exception boost::lock_error during shutdown (`#1656 <https://github.com/ros/ros_comm/issues/1656>`_)
* avoid deadlock in TopicManager (`#1645 <https://github.com/ros/ros_comm/issues/1645>`_)
* use WallTime/WallDuration for waiting for service (`#1638 <https://github.com/ros/ros_comm/issues/1638>`_)
* add missing include path (for bazel workspaces) (`#1636 <https://github.com/ros/ros_comm/issues/1636>`_)
* fix bug in statistics decision making if one should publish (`#1625 <https://github.com/ros/ros_comm/issues/1625>`_)
* add hasStarted() const to WallTimer and SteadyTimer API (`#1565 <https://github.com/ros/ros_comm/issues/1565>`_)
* remove signals from find_package(Boost COMPONENTS ...) (`#1580 <https://github.com/ros/ros_comm/issues/1580>`_)
* fix string error on windows (`#1582 <https://github.com/ros/ros_comm/issues/1582>`_)
* visibility macros update (`#1591 <https://github.com/ros/ros_comm/issues/1591>`_)
* fix race due tounprotected access to callbacks\_ (`#1595 <https://github.com/ros/ros_comm/issues/1595>`_)
* fix nullptr access from Timer().hasStarted() (`#1541 <https://github.com/ros/ros_comm/issues/1541>`_)
* add const specifier to `NodeHandle::param(param_name, default_val)`. (`#1539 <https://github.com/ros/ros_comm/issues/1539>`_)
* update wiki.ros.org URLs (`#1536 <https://github.com/ros/ros_comm/issues/1536>`_)
* fix stamp_age_mean overflow when stamp age very big (`#1526 <https://github.com/ros/ros_comm/issues/1526>`_)
* remove explicit -std=c++11, default to 14
* fix memory error due to missing rosout_disable_topics_generation parameter (`#1507 <https://github.com/ros/ros_comm/issues/1507>`_)
* fix issues when built or run on Windows (`#1466 <https://github.com/ros/ros_comm/issues/1466>`_)

1.16.0 (2022-02-23)
-------------------
* Transport hint fix debug (#21)
* Transporthint tos (#20)
  * Added TOS level to TransportHints
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* fix getNumPublishers() to only count fully connected (#2107)
  There is code which uses getNumPublishers() being positive as a
  guarantee that a call to the remote publisher's publish() is
  guaranteed to send a message to the subscriber. However, because
  connections which have not yet received their header are counted,
  this is not true. One such user of getNumPublishers() is actionlib.
  It relies on getNumPublishers() only returning postive when a publish
  on the result topic would succeed.
  Fix this by only counting connections with received headers. If we
  have received the header, we know the remote publisher is tracking a
  connection back to us and has accepted the connection and that a
  call to publish() on its end will send the message.
  Note that this issue is only relevant for the TCP transport, as the
  UDP transport and intraprocess link have already setup their header
  by the time they are added to the publication links. Since all
  publisher link types update the header, including the caller ID,
  counting those with caller IDs set to non-empty is sufficient to
  solve the issue.
* Replace message assertion with logging in order to have release modes to fail in compilation when msg type mismatches occur (#2096)
* 1.15.9 package.xmls
* 1.15.9
* set call_finished\_ with true for each call inside callFinished (#2074)
  * set call_finished\_ with true for each call inside callFinished
  and also set current_call flags when failure happens
  * Update based on review
  Co-authored-by: Tomoya.Fujita <Tomoya.Fujita@sony.com>
  * set the flag with true again even if it is true already
  Co-authored-by: Tomoya.Fujita <Tomoya.Fujita@sony.com>
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* cached parameter should be unsubscribed (#2068)
  * add unsubscribeCachedParam.
  * unsubscribe all the cached parameters.
  * add const S_string::iterator.
  * delete unnecessary if statement.
  * unsubscribeCachedParam should be called when parameter is deleted.
  * fix parenthesis location.
* fix misspell. (#2066)
* Fix Lost Wake Bug in ROSOutAppender (#2033)
  This fixes a bug in ROSOutAppender that consists of waiting on the condition_variable `queue_condition\_` without checking if the `log_queue\_` is empty.
  In this situation if the `log_queue\_` had some messages that were inserted while `ROSOutAppender::logThread` was publishing other messages, the new messages in the queue won't be published, until another message eventually is added.
  Note that the `notify_all` sent in the destructor would not cause the unpublished messages to get published, as when the `queue_condition\_`  is awaken by this notification, `shutting_down\_` would be true and would cause `ROSOutAppender::logThread`  to return immediately.
  Co-authored-by: Adel Fakih <adel.fakih@avidbots.com>
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* update changelogs
* Changed is_async_connected to use epoll when available (#1983)
  Scalability improvement to remove the 1024 socket limit
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* update changelogs
* fixed Windows build break. (#1961)
* 1.15.6
* update changelogs
* Fix a bug that using a destroyed connection object. (#1950)
  there is a case that accessing a connection object which was destroyed by
  ConnectionManager and TransportSubscriberLink/TransportPublicationLink
  while getting a close event in pollset to call Connection::drop.
* 1.15.5
* update changelogs
* check if async socket connect is success or failure before TransportTCP::read() and TransportTCP::write() (#1954)
  Co-authored-by: hustac <imtoumao@gmail.com>
* Fix bug that connection drop signal related funtion throw a bad_weak … (#1940)
  * Fix bug that connection drop signal related funtion throw a bad_weak exception
  TransportSubscriberLink::onConnectionDropped throws a bad_weak exception
  while call shared_from_this() if TransportSubscriberLink already destroyed
  * Fix dead lock about call_queue_mutex\_ and drop_mutex\_ in two threads
  * Remove check that is not necessary
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix negative numbers in ros statistics (#1531)
  Fix ros/ros_comm#1509
* Remove extra \n in ROS_DEBUG. (#1925)
* 1.15.4
* update changelogs
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* update changelogs
* remove Boost version check since Noetic only targets platforms with 1.67+ (#1903)
* 1.15.2
* update changelogs
* export missing Boost dependency (#1898)
* 1.15.1
* update changelogs
* fix missing boost dependencies (#1895)
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* #961 revisited: Add default ROS_MASTER_URI (#1666)
  * Add default ROS_MASTER_URI
  * roscpp: added getDefaultMasterUri()
  * moved DEFAULT_MASTER_PORT and DEFAULT_MASTER_URI from rospy to rosgraph to make them usable in get_master_uri
  * removed not needed try-catch-block in get_master_uri
  * style of touched lines
  * style of touched lines
  * style of touched lines
  Co-authored-by: Jochen Sprickerhof <github@jochen.sprickerhof.de>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* add default assignment operator for various classes (#1888)
* Bump CMake version to avoid CMP0048 warning (#1869)
* Do not display error message if poll yields EINTR (#1868)
  * Do not display error message if poll yields EINTR
  Closes #1370
  closes TonyRobotics/RoboWare#63
  closes ericsantii/alexa-turtlesim-ros#2
  * Keep abstraction compatibility
* [windows][melodic] portable duration cast (#1882)
* Drop custom implementation of boost::condition_variable to fix busy-wait spinning (#1878)
  * roscpp: fix potential busy-wait loop caused by backported Boost condition_variable (fix ros/ros_comm#1343)
  https://github.com/ros/ros_comm/pull/1014 and https://github.com/ros/ros_comm/pull/1250 introduced a backported
  version of boost::condition_variable, where support for steady (monotonic) clocks has been added in version 1.61.
  But the namespace of the backported version was not changed and the symbol names might clash with the original
  version.
  Because the underlying clock used for the condition_variable is set in the constructor and must be
  consistent with the the expectations within member variables. The compiler might choose to inline one or the
  other or both, and is more likely to do so for optimized Release builds. But if it does not, the symbol ends
  up in the symbol table of roscpp and depending on which other libraries will be linked into the process it
  is unpredictable which of the two versions will be actually called at the end. In case the constructor defined
  in `/usr/include/boost/thread/pthread/condition_variable.hpp` was called and did not configure the internal
  pthread condition variable for monotonic clock, each call to the backported do_wait_until() method with a
  monotonic timestamp will return immediately and hence causes `CallbackQueue::callOne(timeout)` or
  `CallbackQueue::callAvailable(timeout)` to return immediately.
  This patch changes the namespace of the backported condition_variable implementation to boost_161. This
  removes the ambiguity with the original definition if both are used in the same process.
  * roscpp: use boost::condition_variable::wait_for() instead of deprecated timed_wait()
  This fixes ROS timers in combination with 2c18b9f29269ee713c182409666be66af05e06a7. The timer
  callbacks were not called because the TimerManager's thread function blocked indefinitely on
  boost::condition_variable::timed_wait().
  Relative timed_wait() uses the system clock (boost::get_system_time()) unconditionally to
  calculate the absolute timestamp for do_wait_until(). If the condition variable has been
  initialized with BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC, it compares this timestamp
  with the monotonic clock and therefore blocks.
  This issue has been reported in https://svn.boost.org/trac10/ticket/12728 and will not be
  fixed. The timed_wait interface is apparently deprecated.
  * roscpp: do not use boost_161_condition_variable.h on Windows (untested)
  * roscpp: remove specialized implementation of TimerManager<T,D,E>::threadFunc() in steady_timer.cpp
  The updated generic definition in timer_manager.h should do the same with a minor update.
  In all cases we can call boost::condition_variable::wait_until() with an absolute time_point of the respective clock.
  The conversion from system_clock to steady_clock for Time and WallTime is done internally in
  boost::condition_variable::wait_until(lock_type& lock, const chrono::time_point<Clock, Duration>& t).
  * fix namespaces
  * add more explicit namespaces
  * add missing ns
  * roscpp: fixed Boost version check in CMakeLists.txt
  find_package(Boost) has to come before checking the Boost version.
  Otherwise BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC was not defined which
  triggered the assertion in timer_manager.h:240.
  Since Boost 1.67 BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC became the default
  if the platform supports it and the macro is not defined anymore. Instead, check
  for BOOST_THREAD_INTERNAL_CLOCK_IS_MONO.
  * roscpp: replace ROSCPP_BOOST_CONDITION_VARIABLE and ROSCPP_BOOST_CONDITION_VARIABLE_HEADER macros by a typedef in internal_condition_variable.h
  * Remove copy of boost::condition_variable implementation from Boost 1.61 in namespace boost_161
  * Revert some changes in include directives and in CMakeLists.txt to minimize the diff to melodic-devel
  Addresses https://github.com/ros/ros_comm/pull/1878#pullrequestreview-354109870.
  * use wait_for(), remove TimerManagerTraits
  * Revert "use wait_for(), remove TimerManagerTraits"
  This reverts commit 2a67cf6780c32fef3af96fac9da405f4e6eb0298.
  Co-authored-by: Antoine Hoarau <703240+ahoarau@users.noreply.github.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* roscpp: disable rosout via ROSCPP_NO_ROSOUT env var (#1858)
  Introduce the ROSCPP_NO_ROSOUT env var to disable rosout.
  This is more convenient than programmatically passing the `NoRosout` init option,
  and especially needed for nodelets where this option cannot be passed.
* conditionally gaurd sys/socket.h for Windows. (#1876)
* explicit include of socket.h to support FreeBSD (#1864)
  * explicit include of socket.h to support FreeBSD
  * additional explicit include of socket.h to support FreeBSD
* Remove DEBUG statements from getImpl (#1823)
  These statements can potentially be called from within a logging
  function, which causes a "recursive print" warning.
* use c++11 std::snprintf (#1820)
* roscpp: TransportTCP: Allow socket() to return 0 (#1707)
  * roscpp: TransportTCP: Allow socket() to return 0
  In situations that an application has stdin closed, socket() may (and will) return 0. This is totally fine.
  * TransportUDP: fix incoming socket creation
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* roscpp/service_publication: removed int-bool-comparison (#1710)
* add Timer::isValid() const (#1779)
  fix #1650
* Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message. (#1703)
  * Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message.
  Fixes https://github.com/ros/ros_comm/issues/1658.
  * spelling
* Fix segfault in TransportPublisherLink (#1714)
* roscpp/transport_tcp: enable poll event POLLRDHUP to detect dead (#1704)
  connections properly
* roscpp/transport_udp: zero-initialize sockaddr_in object (#1740)
  * roscpp/transport_udp: zero-initialize sockaddr_in object
  * rostcp/transport_udp: zero-initialize sockaddr_in members
* unregisterService returns result of execute("unregisterService") (#1751)
  fix #1744
* fixing string check (#1771)
* Resolve memory leak. (#1503)
  * Resolve memory leak.
  Delete g_rosout_appender explicitly instead of assigning it to NULL.
  Follow deletion with NULL assignment.
  * Deregister g_rosout_appender
  * revert unrelated whitespace change
  * Update init.cpp
  * Increment version number.
  * space in comments
  * Merge
  * Revert "Increment version number."
  This reverts commit 795c8fda33821b635ad30a4828d9071bcc69bed4.
  * Add newer rosconsole dependencies
  * Update rosconsole dependencies in package.xml
  * Sync with upstream completely
  * Add changes
  * Remove deregister function, since it is already done in shutdown().
  * Remove unnecessary .catkin_workspace file.
* /libros/node_handle: alternative way to fix #838 (#1656)
* roscpp/TopicManager: avoid deadlock (#1645)
* roscpp/service: use WallTime/WallDuration for waiting (#1638)
* roscpp: added missin include path (for bazel workspaces) (#1636)
* fixed bug in statistics decision making if one should publish (#1625)
* Add hasStarted() const to WallTimer and SteadyTimer API (#1565)
  * roscpp: copy hasStarted() member function from ros::Timer to ros::WallTimer and ros::SteadyTimer
  ros::Timer::hasStarted() has been added in ros/ros_comm#1464. The same member function should exist in the other
  two timer implementations, too, for completeness.
  * Check for nullptr in WallTimer::hasStarted() and SteadyTimer::hasStarted()
  Analogous to fe9479cdbf0be0caa542c74c7c1fb8229ea8164d (ros/ros_comm#1541).
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix string error on windows (#1582)
  Check if name is empty before read it, or it will cause 'cannot decrement string iterator before begin' error on Windows in debug mode.
* visibility macros update (#1591)
* Fix race due unprotected access to callbacks\_ in roscpp client (#1595)
* Removed nullptr access from Timer().hasStarted() (#1541)
* roscpp: Add const specifier to `NodeHandle::param(param_name, default_val)`. (#1539)
* Update wiki.ros.org URLs (#1536)
* Fix stamp_age_mean overflow when stamp age very big (#1526)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix memory error due to missing rosout_disable_topics_generation para… (#1507)
  * Fix memory error due to missing rosout_disable_topics_generation parameter
  * Initialise disable_topics\_ in constructor of ROSOutAppender
  * Removed security check for rosout_disable_topics_generation parameter in favour of a class initialiser
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Adel Fakih, Alex Moriarty, Arkady Shapkin, Arusekk, Barry Xu, C. Andy Martin, Chen Lihui, Christopher Wecht, Daniel Wang, Dino Hüllmann, Dirk Thomas, Felix Ruess, Gabriel Arjones, Hans-Joachim Krauch, Igor Semenov, Ivor Wanders, Jacob Perron, James Xu, Jeremie Deray, Johannes Meyer, Johnson Shih, Kunal Tyagi, Maarten de Vries, Martin Pecka, Michael Carroll, Michael Johnson, Mikael Arguedas, Mike Purvis, Sean Yen, Shane Loretz, Tahsincan Köse, Victor Lamoine, Zbyněk Winkler, astere-cpr, dodsonmg, foodtooth, randoms, tomoya, wentz89

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* 1.16.0
* Update changelogs
* Transport hint fix debug (#21)
* Transporthint tos (#20)
  * Added TOS level to TransportHints
* 1.15.11
* 1.15.10
* Fix for deadlock issue 1980 (#2121)
  * Add unit test for removing a callback that's being executed.
  This unit test fails until we can remove callbacks that are being executed.
  * Use the marked_for_removal flag if the callback is being executed.
  This fixes a potential deadlock when a timer is being removed that's also being executed.
* fix getNumPublishers() to only count fully connected (#2107)
  There is code which uses getNumPublishers() being positive as a
  guarantee that a call to the remote publisher's publish() is
  guaranteed to send a message to the subscriber. However, because
  connections which have not yet received their header are counted,
  this is not true. One such user of getNumPublishers() is actionlib.
  It relies on getNumPublishers() only returning postive when a publish
  on the result topic would succeed.
  Fix this by only counting connections with received headers. If we
  have received the header, we know the remote publisher is tracking a
  connection back to us and has accepted the connection and that a
  call to publish() on its end will send the message.
  Note that this issue is only relevant for the TCP transport, as the
  UDP transport and intraprocess link have already setup their header
  by the time they are added to the publication links. Since all
  publisher link types update the header, including the caller ID,
  counting those with caller IDs set to non-empty is sufficient to
  solve the issue.
* Replace message assertion with logging in order to have release modes to fail in compilation when msg type mismatches occur (#2096)
* 1.15.9 package.xmls
* 1.15.9
* set call_finished\_ with true for each call inside callFinished (#2074)
  * set call_finished\_ with true for each call inside callFinished
  and also set current_call flags when failure happens
  * Update based on review
  Co-authored-by: Tomoya.Fujita <Tomoya.Fujita@sony.com>
  * set the flag with true again even if it is true already
  Co-authored-by: Tomoya.Fujita <Tomoya.Fujita@sony.com>
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* cached parameter should be unsubscribed (#2068)
  * add unsubscribeCachedParam.
  * unsubscribe all the cached parameters.
  * add const S_string::iterator.
  * delete unnecessary if statement.
  * unsubscribeCachedParam should be called when parameter is deleted.
  * fix parenthesis location.
* fix misspell. (#2066)
* Fix Lost Wake Bug in ROSOutAppender (#2033)
  This fixes a bug in ROSOutAppender that consists of waiting on the condition_variable `queue_condition\_` without checking if the `log_queue\_` is empty.
  In this situation if the `log_queue\_` had some messages that were inserted while `ROSOutAppender::logThread` was publishing other messages, the new messages in the queue won't be published, until another message eventually is added.
  Note that the `notify_all` sent in the destructor would not cause the unpublished messages to get published, as when the `queue_condition\_`  is awaken by this notification, `shutting_down\_` would be true and would cause `ROSOutAppender::logThread`  to return immediately.
  Co-authored-by: Adel Fakih <adel.fakih@avidbots.com>
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* 1.15.8
* update changelogs
* Changed is_async_connected to use epoll when available (#1983)
  Scalability improvement to remove the 1024 socket limit
* Allow mixing latched and unlatched publishers. (#1991)
  * Allow mixing latched and unlatched publishers.
  * Fix isLatched behaviour, add tests.
  * Don't pass unused argument to PublicationPtr.
  * Protect latch accessors with lock.
* 1.15.7
* update changelogs
* fixed Windows build break. (#1961)
* 1.15.6
* update changelogs
* Fix a bug that using a destroyed connection object. (#1950)
  there is a case that accessing a connection object which was destroyed by
  ConnectionManager and TransportSubscriberLink/TransportPublicationLink
  while getting a close event in pollset to call Connection::drop.
* 1.15.5
* update changelogs
* check if async socket connect is success or failure before TransportTCP::read() and TransportTCP::write() (#1954)
  Co-authored-by: hustac <imtoumao@gmail.com>
* Fix bug that connection drop signal related funtion throw a bad_weak … (#1940)
  * Fix bug that connection drop signal related funtion throw a bad_weak exception
  TransportSubscriberLink::onConnectionDropped throws a bad_weak exception
  while call shared_from_this() if TransportSubscriberLink already destroyed
  * Fix dead lock about call_queue_mutex\_ and drop_mutex\_ in two threads
  * Remove check that is not necessary
* roscpp: Multiple latched publishers per process on the same topic (fix #146) (#1544)
  * roscpp: Fix latching of multiple publishers on the same topic.
  * test_roscpp: Add test for multiple latched publishers on same topic.
  * remove obsolete CXX_STANDARD property
  Co-authored-by: Hans-Joachim Krauch <achim.krauch@intermodalics.eu>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix negative numbers in ros statistics (#1531)
  Fix ros/ros_comm#1509
* Remove extra \n in ROS_DEBUG. (#1925)
* 1.15.4
* update changelogs
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* update changelogs
* remove Boost version check since Noetic only targets platforms with 1.67+ (#1903)
* 1.15.2
* update changelogs
* export missing Boost dependency (#1898)
* 1.15.1
* update changelogs
* fix missing boost dependencies (#1895)
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* #961 revisited: Add default ROS_MASTER_URI (#1666)
  * Add default ROS_MASTER_URI
  * roscpp: added getDefaultMasterUri()
  * moved DEFAULT_MASTER_PORT and DEFAULT_MASTER_URI from rospy to rosgraph to make them usable in get_master_uri
  * removed not needed try-catch-block in get_master_uri
  * style of touched lines
  * style of touched lines
  * style of touched lines
  Co-authored-by: Jochen Sprickerhof <github@jochen.sprickerhof.de>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* add default assignment operator for various classes (#1888)
* Bump CMake version to avoid CMP0048 warning (#1869)
* Do not display error message if poll yields EINTR (#1868)
  * Do not display error message if poll yields EINTR
  Closes #1370
  closes TonyRobotics/RoboWare#63
  closes ericsantii/alexa-turtlesim-ros#2
  * Keep abstraction compatibility
* [windows][melodic] portable duration cast (#1882)
* Drop custom implementation of boost::condition_variable to fix busy-wait spinning (#1878)
  * roscpp: fix potential busy-wait loop caused by backported Boost condition_variable (fix ros/ros_comm#1343)
  https://github.com/ros/ros_comm/pull/1014 and https://github.com/ros/ros_comm/pull/1250 introduced a backported
  version of boost::condition_variable, where support for steady (monotonic) clocks has been added in version 1.61.
  But the namespace of the backported version was not changed and the symbol names might clash with the original
  version.
  Because the underlying clock used for the condition_variable is set in the constructor and must be
  consistent with the the expectations within member variables. The compiler might choose to inline one or the
  other or both, and is more likely to do so for optimized Release builds. But if it does not, the symbol ends
  up in the symbol table of roscpp and depending on which other libraries will be linked into the process it
  is unpredictable which of the two versions will be actually called at the end. In case the constructor defined
  in `/usr/include/boost/thread/pthread/condition_variable.hpp` was called and did not configure the internal
  pthread condition variable for monotonic clock, each call to the backported do_wait_until() method with a
  monotonic timestamp will return immediately and hence causes `CallbackQueue::callOne(timeout)` or
  `CallbackQueue::callAvailable(timeout)` to return immediately.
  This patch changes the namespace of the backported condition_variable implementation to boost_161. This
  removes the ambiguity with the original definition if both are used in the same process.
  * roscpp: use boost::condition_variable::wait_for() instead of deprecated timed_wait()
  This fixes ROS timers in combination with 2c18b9f29269ee713c182409666be66af05e06a7. The timer
  callbacks were not called because the TimerManager's thread function blocked indefinitely on
  boost::condition_variable::timed_wait().
  Relative timed_wait() uses the system clock (boost::get_system_time()) unconditionally to
  calculate the absolute timestamp for do_wait_until(). If the condition variable has been
  initialized with BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC, it compares this timestamp
  with the monotonic clock and therefore blocks.
  This issue has been reported in https://svn.boost.org/trac10/ticket/12728 and will not be
  fixed. The timed_wait interface is apparently deprecated.
  * roscpp: do not use boost_161_condition_variable.h on Windows (untested)
  * roscpp: remove specialized implementation of TimerManager<T,D,E>::threadFunc() in steady_timer.cpp
  The updated generic definition in timer_manager.h should do the same with a minor update.
  In all cases we can call boost::condition_variable::wait_until() with an absolute time_point of the respective clock.
  The conversion from system_clock to steady_clock for Time and WallTime is done internally in
  boost::condition_variable::wait_until(lock_type& lock, const chrono::time_point<Clock, Duration>& t).
  * fix namespaces
  * add more explicit namespaces
  * add missing ns
  * roscpp: fixed Boost version check in CMakeLists.txt
  find_package(Boost) has to come before checking the Boost version.
  Otherwise BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC was not defined which
  triggered the assertion in timer_manager.h:240.
  Since Boost 1.67 BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC became the default
  if the platform supports it and the macro is not defined anymore. Instead, check
  for BOOST_THREAD_INTERNAL_CLOCK_IS_MONO.
  * roscpp: replace ROSCPP_BOOST_CONDITION_VARIABLE and ROSCPP_BOOST_CONDITION_VARIABLE_HEADER macros by a typedef in internal_condition_variable.h
  * Remove copy of boost::condition_variable implementation from Boost 1.61 in namespace boost_161
  * Revert some changes in include directives and in CMakeLists.txt to minimize the diff to melodic-devel
  Addresses https://github.com/ros/ros_comm/pull/1878#pullrequestreview-354109870.
  * use wait_for(), remove TimerManagerTraits
  * Revert "use wait_for(), remove TimerManagerTraits"
  This reverts commit 2a67cf6780c32fef3af96fac9da405f4e6eb0298.
  Co-authored-by: Antoine Hoarau <703240+ahoarau@users.noreply.github.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* roscpp: disable rosout via ROSCPP_NO_ROSOUT env var (#1858)
  Introduce the ROSCPP_NO_ROSOUT env var to disable rosout.
  This is more convenient than programmatically passing the `NoRosout` init option,
  and especially needed for nodelets where this option cannot be passed.
* conditionally gaurd sys/socket.h for Windows. (#1876)
* explicit include of socket.h to support FreeBSD (#1864)
  * explicit include of socket.h to support FreeBSD
  * additional explicit include of socket.h to support FreeBSD
* Remove DEBUG statements from getImpl (#1823)
  These statements can potentially be called from within a logging
  function, which causes a "recursive print" warning.
* use c++11 std::snprintf (#1820)
* roscpp: TransportTCP: Allow socket() to return 0 (#1707)
  * roscpp: TransportTCP: Allow socket() to return 0
  In situations that an application has stdin closed, socket() may (and will) return 0. This is totally fine.
  * TransportUDP: fix incoming socket creation
* #1577 revisited: Fix dynamic windowing for Topic Statistics (#1695)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* roscpp/service_publication: removed int-bool-comparison (#1710)
* add Timer::isValid() const (#1779)
  fix #1650
* Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message. (#1703)
  * Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message.
  Fixes https://github.com/ros/ros_comm/issues/1658.
  * spelling
* Fix segfault in TransportPublisherLink (#1714)
* roscpp/transport_tcp: enable poll event POLLRDHUP to detect dead (#1704)
  connections properly
* roscpp/transport_udp: zero-initialize sockaddr_in object (#1740)
  * roscpp/transport_udp: zero-initialize sockaddr_in object
  * rostcp/transport_udp: zero-initialize sockaddr_in members
* unregisterService returns result of execute("unregisterService") (#1751)
  fix #1744
* fixing string check (#1771)
* Resolve memory leak. (#1503)
  * Resolve memory leak.
  Delete g_rosout_appender explicitly instead of assigning it to NULL.
  Follow deletion with NULL assignment.
  * Deregister g_rosout_appender
  * revert unrelated whitespace change
  * Update init.cpp
  * Increment version number.
  * space in comments
  * Merge
  * Revert "Increment version number."
  This reverts commit 795c8fda33821b635ad30a4828d9071bcc69bed4.
  * Add newer rosconsole dependencies
  * Update rosconsole dependencies in package.xml
  * Sync with upstream completely
  * Add changes
  * Remove deregister function, since it is already done in shutdown().
  * Remove unnecessary .catkin_workspace file.
* /libros/node_handle: alternative way to fix #838 (#1656)
* roscpp/TopicManager: avoid deadlock (#1645)
* roscpp/service: use WallTime/WallDuration for waiting (#1638)
* roscpp: added missin include path (for bazel workspaces) (#1636)
* fixed bug in statistics decision making if one should publish (#1625)
* Add hasStarted() const to WallTimer and SteadyTimer API (#1565)
  * roscpp: copy hasStarted() member function from ros::Timer to ros::WallTimer and ros::SteadyTimer
  ros::Timer::hasStarted() has been added in ros/ros_comm#1464. The same member function should exist in the other
  two timer implementations, too, for completeness.
  * Check for nullptr in WallTimer::hasStarted() and SteadyTimer::hasStarted()
  Analogous to fe9479cdbf0be0caa542c74c7c1fb8229ea8164d (ros/ros_comm#1541).
* Remove signals from find_package(Boost COMPONENTS ...) (#1580)
  The packages use signals2, not signals. Only boost libraries with
  compiled code should be passed to find_package(Boost COMPONENTS ...),
  and the signals2 library has always been header only.
  Boost 1.69 has removed the deprecated signals library, so the otherwise
  useless but harmless `signals` component now breaks the build.
* Fix string error on windows (#1582)
  Check if name is empty before read it, or it will cause 'cannot decrement string iterator before begin' error on Windows in debug mode.
* visibility macros update (#1591)
* Fix race due unprotected access to callbacks\_ in roscpp client (#1595)
* Removed nullptr access from Timer().hasStarted() (#1541)
* roscpp: Add const specifier to `NodeHandle::param(param_name, default_val)`. (#1539)
* Update wiki.ros.org URLs (#1536)
* Fix stamp_age_mean overflow when stamp age very big (#1526)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix memory error due to missing rosout_disable_topics_generation para… (#1507)
  * Fix memory error due to missing rosout_disable_topics_generation parameter
  * Initialise disable_topics\_ in constructor of ROSOutAppender
  * Removed security check for rosout_disable_topics_generation parameter in favour of a class initialiser
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Adel Fakih, Alex Moriarty, Arkady Shapkin, Arusekk, Barry Xu, C. Andy Martin, Chen Lihui, Christopher Wecht, Daniel Wang, Dino Hüllmann, Dirk Thomas, Felix Ruess, Gabriel Arjones, Gary Servin, Hans-Joachim Krauch, Igor Semenov, Ivor Wanders, Jacob Perron, James Xu, Jeremie Deray, Johannes Meyer, Johnson Shih, Kunal Tyagi, Maarten de Vries, Martin Pecka, Michael Carroll, Michael Johnson, Mikael Arguedas, Mike Purvis, Sean Yen, Shane Loretz, Tahsincan Köse, Victor Lamoine, Zbyněk Winkler, astere-cpr, dodsonmg, foodtooth, randoms, tomoya, wentz89

1.14.3 (2018-08-06)
-------------------
* add hasStarted() to Timer API (`#1464 <https://github.com/ros/ros_comm/issues/1464>`_)
* fix compiler warnings about unused variables (`#1428 <https://github.com/ros/ros_comm/issues/1428>`_)

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* force a rebuild of the pollset on flag changes (`#1393 <https://github.com/ros/ros_comm/issues/1393>`_)
* fix integer overflow for oneshot timers (`#1382 <https://github.com/ros/ros_comm/issues/1382>`_)
* convert the period standard deviation in StatisticsLogger to Duration at the end (`#1361 <https://github.com/ros/ros_comm/issues/1361>`_)
* add time when timer expired to timer events (`#1130 <https://github.com/ros/ros_comm/issues/1130>`_)
* replace DCL pattern with static variable (`#1365 <https://github.com/ros/ros_comm/issues/1365>`_)
* add parameter to stop clients from generating rosout topics list (`#1241 <https://github.com/ros/ros_comm/issues/1241>`_)

1.13.6 (2018-02-05)
-------------------
* avoid recreating poll set (`#1281 <https://github.com/ros/ros_comm/pull/1281>`_)
* switch to using epoll (`#1217 <https://github.com/ros/ros_comm/pull/1217>`_)
* monotonic clock for callback queue timeouts (`#1250 <https://github.com/ros/ros_comm/pull/1250>`_)
* fix IPv6 initialization order (`#1262 <https://github.com/ros/ros_comm/issues/1262>`_)
* changed error message for single threaded spinner  (`#1164 <https://github.com/ros/ros_comm/pull/1164>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* avoid unused parameter warning in TransportTCP (`#1195 <https://github.com/ros/ros_comm/issues/1195>`_)
* check if socket options are available before using them (`#1172 <https://github.com/ros/ros_comm/issues/1172>`_)

1.13.2 (2017-08-15)
-------------------
* only use CLOCK_MONOTONIC if not on OS X (`#1142 <https://github.com/ros/ros_comm/issues/1142>`_)
* xmlrpc_manager: use SteadyTime for timeout (`#1134 <https://github.com/ros/ros_comm/issues/1134>`_)
* ignore headers with zero stamp in statistics (`#1127 <https://github.com/ros/ros_comm/issues/1127>`_)

1.13.1 (2017-07-27)
-------------------
* add SteadyTimer, used in TimerManager (`#1014 <https://github.com/ros/ros_comm/issues/1014>`_)
* include missing header for writev() (`#1105 <https://github.com/ros/ros_comm/pull/1105>`_)
* clean the namespace to get rid of double or trailing forward slashes (`#1100 <https://github.com/ros/ros_comm/issues/1100>`_)
* add missing mutex lock for publisher links (`#1090 <https://github.com/ros/ros_comm/pull/1090>`_)
* fix race condition that lead to miss first message (`#1058 <https://github.com/ros/ros_comm/issues/1058>`_)
* fix bug in transport_tcp on Windows (`#1050 <https://github.com/ros/ros_comm/issues/1050>`_)
* add subscriber to connection log messages (`#1023 <https://github.com/ros/ros_comm/issues/1023>`_)
* avoid deleting XmlRpcClient while being used in another thread (`#1013 <https://github.com/ros/ros_comm/issues/1013>`_)

1.13.0 (2017-02-22)
-------------------
* remove support for multiple spinners on the same queue which existed only for backward compatibily (`#988 <https://github.com/ros/ros_comm/pull/988>`_)

1.12.7 (2017-02-17)
-------------------
* move connection specific log message to new name roscpp_internal.connections (`#980 <https://github.com/ros/ros_comm/pull/980>`_)
* move headers to include/xmlrpcpp (`#962 <https://github.com/ros/ros_comm/issues/962>`_)
* fix UDP block number when EAGAIN or EWOULDBLOCK (`#957 <https://github.com/ros/ros_comm/issues/957>`_)
* fix return code of master execute function (`#938 <https://github.com/ros/ros_comm/pull/938>`_)
* change WallTimerEvent from class to struct (`#924 <https://github.com/ros/ros_comm/pull/924>`_)

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* fix multi-threaded spinning (`#867 <https://github.com/ros/ros_comm/pull/867>`_)
* fix static destruction order (`#871 <https://github.com/ros/ros_comm/pull/871>`_)
* throw exception on ros::init with empty node name (`#894 <https://github.com/ros/ros_comm/pull/894>`_)
* improve debug message when queue is full (`#818 <https://github.com/ros/ros_comm/issues/818>`_)

1.12.2 (2016-06-03)
-------------------
* improve stacktrace for exceptions thrown in callbacks (`#811 <https://github.com/ros/ros_comm/pull/811>`_)
* fix segfault if creating outgoing UDP transport fails (`#807 <https://github.com/ros/ros_comm/pull/807>`_)

1.12.1 (2016-04-18)
-------------------
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------
* improve TopicManager::instance (`#770 <https://github.com/ros/ros_comm/issues/770>`_)
* change return value of param() to bool (`#753 <https://github.com/ros/ros_comm/issues/753>`_)

1.11.18 (2016-03-17)
--------------------
* fix CMake warning about non-existing targets

1.11.17 (2016-03-11)
--------------------
* fix order of argument in SubscriberLink interface to match actual implemenation (`#701 <https://github.com/ros/ros_comm/issues/701>`_)
* add method for getting all the parameters from the parameter server as implemented in the rospy client (`#739 <https://github.com/ros/ros_comm/issues/739>`_)
* use boost::make_shared instead of new for constructing boost::shared_ptr (`#740 <https://github.com/ros/ros_comm/issues/740>`_)
* fix max elements param for statistics window (`#750 <https://github.com/ros/ros_comm/issues/750>`_)
* improve NodeHandle constructor documentation (`#692 <https://github.com/ros/ros_comm/issues/692>`_)

1.11.16 (2015-11-09)
--------------------
* add getROSArg function (`#694 <https://github.com/ros/ros_comm/pull/694>`_)

1.11.15 (2015-10-13)
--------------------
* fix crash in onRetryTimer() callback (`#577 <https://github.com/ros/ros_comm/issues/577>`_)

1.11.14 (2015-09-19)
--------------------
* add optional reset argument to Timer::setPeriod() (`#590 <https://github.com/ros/ros_comm/issues/590>`_)
* add getParam() and getParamCached() for float (`#621 <https://github.com/ros/ros_comm/issues/621>`_, `#623 <https://github.com/ros/ros_comm/issues/623>`_)
* use explicit bool cast to compile with C++11 (`#632 <https://github.com/ros/ros_comm/pull/632>`_)

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* fix memory leak in transport constructor (`#570 <https://github.com/ros/ros_comm/pull/570>`_)
* fix computation of stddev in statistics (`#556 <https://github.com/ros/ros_comm/pull/556>`_)
* fix empty connection header topic (`#543 <https://github.com/ros/ros_comm/issues/543>`_)
* alternative API to get parameter values (`#592 <https://github.com/ros/ros_comm/pull/592>`_)
* add getCached() for float parameters (`#584 <https://github.com/ros/ros_comm/pull/584>`_)

1.11.10 (2014-12-22)
--------------------
* fix various defects reported by coverity
* fix comment (`#529 <https://github.com/ros/ros_comm/issues/529>`_)
* improve Android support (`#518 <https://github.com/ros/ros_comm/pull/518>`_)

1.11.9 (2014-08-18)
-------------------
* add accessor to expose whether service is persistent (`#489 <https://github.com/ros/ros_comm/issues/489>`_)
* populate delivered_msgs field of TopicStatistics message (`#486 <https://github.com/ros/ros_comm/issues/486>`_)

1.11.8 (2014-08-04)
-------------------
* fix C++11 compatibility issue (`#483 <https://github.com/ros/ros_comm/issues/483>`_)

1.11.7 (2014-07-18)
-------------------
* fix segfault due to accessing a NULL pointer for some network interfaces (`#465 <https://github.com/ros/ros_comm/issues/465>`_) (regression from 1.11.6)

1.11.6 (2014-07-10)
-------------------
* check ROS_HOSTNAME for localhost / ROS_IP for 127./::1 and prevent connections from other hosts in that case (`#452 <https://github.com/ros/ros_comm/issues/452>`_)

1.11.5 (2014-06-24)
-------------------
* improve handling dropped connections (`#434 <https://github.com/ros/ros_comm/issues/434>`_)
* add header needed for Android (`#441 <https://github.com/ros/ros_comm/issues/441>`_)
* fix typo for parameter used for statistics (`#448 <https://github.com/ros/ros_comm/issues/448>`_)

1.11.4 (2014-06-16)
-------------------

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* update API to use boost::signals2 (`#267 <https://github.com/ros/ros_comm/issues/267>`_)
* only update param cache when being subscribed (`#351 <https://github.com/ros/ros_comm/issues/351>`_)
* ensure to remove delete parameters completely
* invalidate cached parent parameters when namespace parameter is set / changes (`#352 <https://github.com/ros/ros_comm/issues/352>`_)
* add optional topic/connection statistics (`#398 <https://github.com/ros/ros_comm/issues/398>`_)
* add transport information in SlaveAPI::getBusInfo() for roscpp & rospy (`#328 <https://github.com/ros/ros_comm/issues/328>`_)
* add AsyncSpinner::canStart() to check if a spinner can be started

1.11.0 (2014-03-04)
-------------------
* allow getting parameters with name '/' (`#313 <https://github.com/ros/ros_comm/issues/313>`_)
* support for /clock remapping (`#359 <https://github.com/ros/ros_comm/issues/359>`_)
* suppress boost::signals deprecation warning (`#362 <https://github.com/ros/ros_comm/issues/362>`_)
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* remove use of __connection header

1.9.54 (2014-01-27)
-------------------
* fix return value of pubUpdate() (`#334 <https://github.com/ros/ros_comm/issues/334>`_)
* fix handling optional third xml rpc response argument (`#335 <https://github.com/ros/ros_comm/issues/335>`_)

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* move several client library independent parts from ros_comm into roscpp_core, split rosbag storage specific stuff from client library usage (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* add missing version dependency on roscpp_core stuff (`#299 <https://github.com/ros/ros_comm/issues/299>`_)
* remove log4cxx dependency from roscpp, using new agnostic interface from rosconsole
* fix compile problem with gcc 4.4 (`#302 <https://github.com/ros/ros_comm/issues/302>`_)
* fix clang warnings
* fix usage of boost include directories

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------
* add rosparam getter/setter for std::vector and std::map (`#279 <https://github.com/ros/ros_comm/issues/279>`_)

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------
* improve handling of UDP transport, when fragmented packets are lost or arive out-of-order the connection is not dropped anymore, onle a single message is lost (`#226 <https://github.com/ros/ros_comm/issues/226>`_)
* fix missing generation of constant definitions for services (`ros/gencpp#2 <https://github.com/ros/gencpp/issues/2>`_)
* fix restoring thread context when callback throws an exception (`#219 <https://github.com/ros/ros_comm/issues/219>`_)
* fix calling PollManager::shutdown() repeatedly (`#217 <https://github.com/ros/ros_comm/issues/217>`_)

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* improve speed of message generation in dry packages (`#183 <https://github.com/ros/ros_comm/issues/183>`_)
* fix roscpp service call deadlock (`#149 <https://github.com/ros/ros_comm/issues/149>`_)
* fix freezing service calls when returning false (`#168 <https://github.com/ros/ros_comm/issues/168>`_)
* fix error message publishing wrong message type (`#178 <https://github.com/ros/ros_comm/issues/178>`_)
* fix missing explicit dependency on pthread (`#135 <https://github.com/ros/ros_comm/issues/135>`_)
* fix compiler warning about wrong comparison of message md5 hashes (`#165 <https://github.com/ros/ros_comm/issues/165>`_)

1.9.41 (2013-01-24)
-------------------
* allow sending data exceeding 2GB in chunks (`#4049 <https://code.ros.org/trac/ros/ticket/4049>`_)
* update getParam() doc (`#1460 <https://code.ros.org/trac/ros/ticket/1460>`_)
* add param::get(float) (`#3754 <https://code.ros.org/trac/ros/ticket/3754>`_)
* update inactive assert when publishing message with md5sum "*", update related tests (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)
* fix ros master retry timeout (`#4024 <https://code.ros.org/trac/ros/ticket/4024>`_)
* fix inactive assert when publishing message with wrong type (`#3714 <https://code.ros.org/trac/ros/ticket/3714>`_)

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
