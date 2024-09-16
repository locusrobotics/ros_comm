^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosmaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.11 (2021-04-06)
--------------------

1.15.10 (2021-03-18)
--------------------

1.15.9 (2020-10-16)
-------------------
* Update maintainers (`#2075 <https://github.com/ros/ros_comm/issues/2075>`_)
* Fix case where accessing cached parameters shuts down another node (`#2068 <https://github.com/ros/ros_comm/issues/2068>`_)
* Fix spelling (`#2066 <https://github.com/ros/ros_comm/issues/2066>`_)
* Contributors: Shane Loretz, tomoya

1.15.8 (2020-07-23)
-------------------
* improve shutdown message with duplicate node name (`#1992 <https://github.com/ros/ros_comm/issues/1992>`_)

1.15.7 (2020-05-28)
-------------------

1.15.6 (2020-05-21)
-------------------

1.15.5 (2020-05-15)
-------------------

1.15.4 (2020-03-19)
-------------------

1.15.3 (2020-02-28)
-------------------

1.15.2 (2020-02-25)
-------------------

1.15.1 (2020-02-24)
-------------------
* use setuptools instead of distutils (`#1870 <https://github.com/ros/ros_comm/issues/1870>`_)

1.15.0 (2020-02-21)
-------------------

1.14.4 (2020-02-20)
-------------------
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* use thread local storage for caching instances of ServerProxy (`#1732 <https://github.com/ros/ros_comm/issues/1732>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* fix issue occurring during alternating calls of getParamCached and setParam (`#1439 <https://github.com/ros/ros_comm/issues/1439>`_)
* fix docstring in unregisterSubscriber (`#1553 <https://github.com/ros/ros_comm/issues/1553>`_)
* set correctly typed @apivalidate default return values (`#1472 <https://github.com/ros/ros_comm/issues/1472>`_)

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* 1.15.9
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
* 1.15.8
* update changelogs
* Add which node has been registered with the same name (#1992)
  * Add which node has been registered with the same name
  Was looking through logs of several nodes that had been launched at the same time and it was getting hard to tell which node was being duplicated. This would allow you do to see the name of the node that caused the issue
  * switch to shutdown_node_task
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* 1.15.4
* update changelogs
* 1.15.3
* update changelogs
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* Use setuptools instead of distutils (#1870)
  * Use setuptools instead of distutils
  * Remove explicit setuptools dependency
  * revert unrelated package format changes
  * restore xml version
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* Use thread local storage for caching instances of ServerProxy (#1732)
  xmlrpc.client.ServerProxy is not thread safe. See
  https://bugs.python.org/issue6907
  The symptom of this bug is exceptions in the publisherUpdate
  logged in the master.log. For example:
  [rosmaster.threadpool][ERROR] : Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/threadpool.py", line 218, in run
  result = cmd(*args)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/master_api.py", line 210, in publisher_update_task
  ret = xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 68, in xmlrpcapi
  close_half_closed_sockets()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 79, in close_half_closed_sockets
  state = transport._connection[1].sock.getsockopt(socket.SOL_TCP, socket.TCP_INFO)
  File "/usr/lib/python2.7/socket.py", line 228, in meth
  return getattr(self._sock,name)(*args)
  File "/usr/lib/python2.7/socket.py", line 174, in _dummy
  raise error(EBADF, 'Bad file descriptor')
  error: [Errno 9] Bad file descriptor
  Some subscribers get the update but some do not. For example, the topic
  is recorded in a rosbag but not received by nodes that depend on it.
  Issue: https://github.com/ros/ros_comm/issues/1523
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Fixed docstring in unregisterSubscriber (#1553)
  Fixes #1508.
* Setting correctly typed @apivalidate default return values (#1472)
* Contributors: BoukeKromTNO, Carl Saldanha, Christopher Wecht, Dirk Thomas, Jacob Perron, John Fettig, Kostya, Shane Loretz, tomoya

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* 1.15.9
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
* 1.15.8
* update changelogs
* Add which node has been registered with the same name (#1992)
  * Add which node has been registered with the same name
  Was looking through logs of several nodes that had been launched at the same time and it was getting hard to tell which node was being duplicated. This would allow you do to see the name of the node that caused the issue
  * switch to shutdown_node_task
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* 1.15.4
* update changelogs
* 1.15.3
* update changelogs
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* Use setuptools instead of distutils (#1870)
  * Use setuptools instead of distutils
  * Remove explicit setuptools dependency
  * revert unrelated package format changes
  * restore xml version
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* Use thread local storage for caching instances of ServerProxy (#1732)
  xmlrpc.client.ServerProxy is not thread safe. See
  https://bugs.python.org/issue6907
  The symptom of this bug is exceptions in the publisherUpdate
  logged in the master.log. For example:
  [rosmaster.threadpool][ERROR] : Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/threadpool.py", line 218, in run
  result = cmd(*args)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/master_api.py", line 210, in publisher_update_task
  ret = xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 68, in xmlrpcapi
  close_half_closed_sockets()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 79, in close_half_closed_sockets
  state = transport._connection[1].sock.getsockopt(socket.SOL_TCP, socket.TCP_INFO)
  File "/usr/lib/python2.7/socket.py", line 228, in meth
  return getattr(self._sock,name)(*args)
  File "/usr/lib/python2.7/socket.py", line 174, in _dummy
  raise error(EBADF, 'Bad file descriptor')
  error: [Errno 9] Bad file descriptor
  Some subscribers get the update but some do not. For example, the topic
  is recorded in a rosbag but not received by nodes that depend on it.
  Issue: https://github.com/ros/ros_comm/issues/1523
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Fixed docstring in unregisterSubscriber (#1553)
  Fixes #1508.
* Setting correctly typed @apivalidate default return values (#1472)
* Contributors: BoukeKromTNO, Carl Saldanha, Christopher Wecht, Dirk Thomas, Gary Servin, Jacob Perron, John Fettig, Kostya, Shane Loretz, tomoya

1.19.0 (2023-09-25)
-------------------
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* 1.15.9
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
* 1.15.8
* update changelogs
* Add which node has been registered with the same name (#1992)
  * Add which node has been registered with the same name
  Was looking through logs of several nodes that had been launched at the same time and it was getting hard to tell which node was being duplicated. This would allow you do to see the name of the node that caused the issue
  * switch to shutdown_node_task
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* 1.15.4
* update changelogs
* 1.15.3
* update changelogs
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* Use setuptools instead of distutils (#1870)
  * Use setuptools instead of distutils
  * Remove explicit setuptools dependency
  * revert unrelated package format changes
  * restore xml version
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* Use thread local storage for caching instances of ServerProxy (#1732)
  xmlrpc.client.ServerProxy is not thread safe. See
  https://bugs.python.org/issue6907
  The symptom of this bug is exceptions in the publisherUpdate
  logged in the master.log. For example:
  [rosmaster.threadpool][ERROR] : Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/threadpool.py", line 218, in run
  result = cmd(*args)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/master_api.py", line 210, in publisher_update_task
  ret = xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 68, in xmlrpcapi
  close_half_closed_sockets()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 79, in close_half_closed_sockets
  state = transport._connection[1].sock.getsockopt(socket.SOL_TCP, socket.TCP_INFO)
  File "/usr/lib/python2.7/socket.py", line 228, in meth
  return getattr(self._sock,name)(*args)
  File "/usr/lib/python2.7/socket.py", line 174, in _dummy
  raise error(EBADF, 'Bad file descriptor')
  error: [Errno 9] Bad file descriptor
  Some subscribers get the update but some do not. For example, the topic
  is recorded in a rosbag but not received by nodes that depend on it.
  Issue: https://github.com/ros/ros_comm/issues/1523
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Fixed docstring in unregisterSubscriber (#1553)
  Fixes #1508.
* Setting correctly typed @apivalidate default return values (#1472)
* Contributors: BoukeKromTNO, Carl Saldanha, Christopher Wecht, Dirk Thomas, Gary Servin, Jacob Perron, John Fettig, Kostya, Shane Loretz, tomoya

Forthcoming
-----------

1.21.0 (2024-06-17)
-------------------

1.20.0 (2024-02-02)
-------------------
* Remove raise (#35)
  * remove raise
  * fix
  * remove
* 1.19.0
* Update changelogs
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* 1.15.9
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
* 1.15.8
* update changelogs
* Add which node has been registered with the same name (#1992)
  * Add which node has been registered with the same name
  Was looking through logs of several nodes that had been launched at the same time and it was getting hard to tell which node was being duplicated. This would allow you do to see the name of the node that caused the issue
  * switch to shutdown_node_task
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* 1.15.4
* update changelogs
* 1.15.3
* update changelogs
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* Use setuptools instead of distutils (#1870)
  * Use setuptools instead of distutils
  * Remove explicit setuptools dependency
  * revert unrelated package format changes
  * restore xml version
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* Use thread local storage for caching instances of ServerProxy (#1732)
  xmlrpc.client.ServerProxy is not thread safe. See
  https://bugs.python.org/issue6907
  The symptom of this bug is exceptions in the publisherUpdate
  logged in the master.log. For example:
  [rosmaster.threadpool][ERROR] : Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/threadpool.py", line 218, in run
  result = cmd(*args)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/master_api.py", line 210, in publisher_update_task
  ret = xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 68, in xmlrpcapi
  close_half_closed_sockets()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rosmaster/util.py", line 79, in close_half_closed_sockets
  state = transport._connection[1].sock.getsockopt(socket.SOL_TCP, socket.TCP_INFO)
  File "/usr/lib/python2.7/socket.py", line 228, in meth
  return getattr(self._sock,name)(*args)
  File "/usr/lib/python2.7/socket.py", line 174, in _dummy
  raise error(EBADF, 'Bad file descriptor')
  error: [Errno 9] Bad file descriptor
  Some subscribers get the update but some do not. For example, the topic
  is recorded in a rosbag but not received by nodes that depend on it.
  Issue: https://github.com/ros/ros_comm/issues/1523
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Fixed issue occuring during alternating calls of getParamCached and setParam (#1439)
  * test_roscpp/params/added getParamCachedSetParamLoop
  * rosmaster: set_param: the not update the caller!
  * rosmaster: set_param: do not update the caller more fine grained
  * /rosmaster/paramserver/compute_params_update, apply filter only if caller_id_to_ignore is not None
  * /test_rospy/talker: set publishers queue_size to supress warning
  * /test_rospy/sub_to_multple_pubs: moved listener up to avoid warnings
  * refactor for readability
  * pep8
* Fixed docstring in unregisterSubscriber (#1553)
  Fixes #1508.
* Setting correctly typed @apivalidate default return values (#1472)
* Contributors: BoukeKromTNO, Carl Saldanha, Christopher Wecht, Dirk Thomas, Gary Servin, Jacob Perron, John Fettig, Joshua Wallace, Kostya, Shane Loretz, tomoya

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------

1.13.6 (2018-02-05)
-------------------
* add TCP_INFO availability check (`#1211 <https://github.com/ros/ros_comm/issues/1211>`_)
* replace Thread.setDaemon() using new API (`#1276 <https://github.com/ros/ros_comm/issues/1276>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------
* catch exception with `socket.TCP_INFO` on WSL (`#1212 <https://github.com/ros/ros_comm/issues/1212>`_, regression from 1.13.1)

1.13.3 (2017-10-25)
-------------------
* add --set-master-logger-level option for 'rosmaster' to output LOG_API (`#1180 <https://github.com/ros/ros_comm/issues/1180>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* close CLOSE_WAIT sockets by default (`#1104 <https://github.com/ros/ros_comm/issues/1104>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* add more logging to publisher update calls (`#979 <https://github.com/ros/ros_comm/issues/979>`_)

1.12.6 (2016-10-26)
-------------------

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------

1.12.2 (2016-06-03)
-------------------

1.12.1 (2016-04-18)
-------------------
* use defusedxml to prevent common xml issues (`#782 <https://github.com/ros/ros_comm/pull/782>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------

1.11.16 (2015-11-09)
--------------------
* add `-w` and `-t` options (`#687 <https://github.com/ros/ros_comm/pull/687>`_)

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

1.11.10 (2014-12-22)
--------------------
* fix closing sockets properly on node shutdown (`#495 <https://github.com/ros/ros_comm/issues/495>`_)

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

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_, `#429 <https://github.com/ros/ros_comm/issues/429>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------

1.10.0 (2014-02-11)
-------------------

1.9.54 (2014-01-27)
-------------------

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
