^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.16.0 (2020-11-10)
-------------------

1.15.0 (2020-10-02)
-------------------
* Initialize publisher/subscriber options in impl constructors (`#19 <https://github.com/locusrobotics/ros_comm/issues/19>`_)
  Initialize publisher/subscriber options in impl constructors
  Prevents misconfigured connections from being added to a topic before
  topic configuration is complete.
* Use cached parameter for rosout_disable_topics_generation
* more Python 3 compatibility (`#1795 <https://github.com/locusrobotics/ros_comm/issues/1795>`_)
  * avoid using nose.tools without dependency being declared
  * seek(0)
  * subprocess decode
  * import urlparse
  * fix hash arg encode
  * print function
  * replace tabs used for indenting Python code with spaces
* fix line endings to be LF (`#1794 <https://github.com/locusrobotics/ros_comm/issues/1794>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/locusrobotics/ros_comm/issues/1792>`_)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* `#1577 <https://github.com/locusrobotics/ros_comm/issues/1577>`_ revisited: Fix dynamic windowing for Topic Statistics (`#1695 <https://github.com/locusrobotics/ros_comm/issues/1695>`_)
  * Add failing tests for topic statistics frequency for rospy and roscpp
  * Fix TopicStatistics dynamic windowing to adjust evaluation frequency in the right direction
  * test_roscpp: fixed topic_statistic_frequency
  * test_roscpp/topic_statistic_frequency: cleanup
* Do not raise socket exception during shutdown (`#1720 <https://github.com/locusrobotics/ros_comm/issues/1720>`_)
* Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message. (`#1703 <https://github.com/locusrobotics/ros_comm/issues/1703>`_)
  * Added possibility to pass rospy.Duration as timeout to wait_for_service and wait_for_message.
  Fixes https://github.com/ros/ros_comm/issues/1658.
  * spelling
* added is_legal_remap() to rosgraph to make remap-detection more precise (`#1683 <https://github.com/locusrobotics/ros_comm/issues/1683>`_)
  * added is_legal_remap() to rosgraph
  * test_rospy/test_rospy_client.py: fixed failing test
  * removed unrelated change
* Add missing comma in the list of strings (`#1760 <https://github.com/locusrobotics/ros_comm/issues/1760>`_)
  The missing comma will implicitly concatenate the string "FATAL" and "is_shutdown" together
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
* Fix error handling for Topic constructor (`#1701 <https://github.com/locusrobotics/ros_comm/issues/1701>`_)
  There is no s variable in scope - and we clearly wanna display
  reg_type
* Make sigterm handling python3 compatible. (`#1559 <https://github.com/locusrobotics/ros_comm/issues/1559>`_)
* Update wiki.ros.org URLs (`#1536 <https://github.com/locusrobotics/ros_comm/issues/1536>`_)
* Added missing lock
* Use cache when possible
* Fixed behavior for unset keys
* Fixes for failing test
* Fixes for broken tests
* Avoid unnecessary whitespace change
* added get_param_cached
* show connection info on rosnode info (`#1497 <https://github.com/locusrobotics/ros_comm/issues/1497>`_)
* import socket, threading in udpros.py (`#1494 <https://github.com/locusrobotics/ros_comm/issues/1494>`_)
  * import socket in udpros.py
  Avoids a host of undefined names:
  [flake8](http://flake8.pycqa.org) testing of https://github.com/ros/ros_comm on Python 3.6.3
  $ __flake8 . --count --select=E901,E999,F821,F822,F823 --show-source --statistics\_\_
  ```
  ./clients/rospy/src/rospy/names.py:62:30: F821 undefined name 'basestring'
  return isinstance(s, basestring) #Python 2.x
  ^
  ./clients/rospy/src/rospy/impl/tcpros_service.py:72:30: F821 undefined name 'basestring'
  return isinstance(s, basestring) #Python 2.x
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:72:17: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:72:31: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:72:48: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:74:17: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:74:31: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:74:47: F821 undefined name 'socket'
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:79:9: F821 undefined name 'threading'
  threading.start_new_thread(self.run, ())
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:112:34: F821 undefined name 'UDPROS'
  if protocol_params[0] != UDPROS:
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:128:21: F821 undefined name 'UDPTransport'
  transport = UDPTransport(protocol, topic_name, sub.receive_callback)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:128:34: F821 undefined name 'protocol'
  transport = UDPTransport(protocol, topic_name, sub.receive_callback)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:144:28: F821 undefined name 'UDPROS'
  return protocol == UDPROS
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:150:18: F821 undefined name 'UDPROS'
  return [[UDPROS]]
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:166:34: F821 undefined name 'UDPROS'
  if protocol_params[0] != UDPROS:
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:167:76: F821 undefined name 'protocol'
  return 0, "Internal error: protocol does not match UDPROS: %s"%protocol, []
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:171:29: F821 undefined name 'UDPROS'
  return 1, "ready", [UDPROS]
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:222:17: F821 undefined name '_configure_pub_socket'
  _configure_pub_socket(sock, tcp_nodelay)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:223:28: F821 undefined name 'TCPROSPub'
  protocol = TCPROSPub(resolved_topic_name, topic.data_class, is_latch=topic.is_latch, headers=topic.headers)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:224:29: F821 undefined name 'TCPROSTransport'
  transport = TCPROSTransport(protocol, resolved_topic_name)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:249:19: F821 undefined name 'TransportInitError'
  raise TransportInitError("Unable to initialize transport: name is not set")
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:267:9: F821 undefined name 'serialize_message'
  serialize_message(self.write_buff, seq, msg)
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:299:9: F821 undefined name 'self'
  self(UDPROSTransport, self).close()
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:299:31: F821 undefined name 'self'
  self(UDPROSTransport, self).close()
  ^
  ./clients/rospy/src/rospy/impl/udpros.py:301:9: F821 undefined name 'self'
  self.done = True
  ^
  ./test/test_rosmaster/test/testMaster.py:86:15: E999 SyntaxError: invalid syntax
  print graph[1]
  ^
  ./test/test_rosmaster/test/testSlave.py:277:27: E999 SyntaxError: invalid syntax
  print "Testing", test
  ^
  ./test/test_rosmaster/test/client_verification/test_slave_api.py:127:30: E999 SyntaxError: invalid syntax
  print "[%s] API  = %s"%(self.test_node, self.node_api)
  ^
  ./test/test_rospy/test/rostest/test_node.py:58:41: F821 undefined name 'Time'
  new_data.header.stamp = Time(1234, 5678)
  ^
  ./test/test_rospy/test/unit/test_genmsg_py.py:266:20: F821 undefined name 'long'
  maxp = long(math.pow(2, w-1)) - 1
  ^
  ./test/test_rospy/test/unit/test_genmsg_py.py:267:21: F821 undefined name 'long'
  maxn = -long(math.pow(2, w-1)) + 1
  ^
  ./test/test_rospy/test/unit/test_genmsg_py.py:303:79: F821 undefined name 'w'
  self.fail("check_types should have noted sign error[%s]: %s"%(w, cls.__name_\_))
  ^
  ./test/test_rospy/test/unit/test_genmsg_py.py:309:20: F821 undefined name 'long'
  maxp = long(math.pow(2, w)) - 1
  ^
  ./test/test_rospy/test/unit/test_rospy_rostime.py:239:33: F821 undefined name 'Time'
  v = Duration(1,0) + Time(1, 0)
  ^
  ./test/test_rospy/test/unit/test_rospy_rostime.py:275:34: F821 undefined name 'Time'
  v = Duration(1, 0) - Time(1,0)
  ^
  ./test/test_rosservice/test/test_rosservice_command_line_offline.py:94:40: F821 undefined name 'NAME'
  rostest.unitrun('test_rosservice', NAME, TestRosserviceOffline, sys.argv, coverage_packages=[])
  ^
  ./tools/rosbag/scripts/bag2png.py:51:42: F821 undefined name 'ma'
  ma, image_data = msg.uint8_data, ma.data
  ^
  ./tools/rosbag/scripts/fix_msg_defs.py:64:31: F821 undefined name 'roslib'
  systype = roslib.message.get_message_class(msg[0])
  ^
  ./tools/rosbag/scripts/makerule.py:136:32: F821 undefined name 'raw_input'
  new_type = raw_input('>')
  ^
  ./tools/rosbag/scripts/makerule.py:140:36: F821 undefined name 'raw_input'
  new_type = raw_input('>')
  ^
  ./tools/rosbag/src/rosbag/migration.py:1115:100: F821 undefined name 'msg_from'
  raise BagMigrationException("Migrate called, but no migration path from [%s] to [%s]"%(msg_from._type, msg_to._type))
  ^
  ./tools/rosbag/src/rosbag/migration.py:1115:116: F821 undefined name 'msg_to'
  raise BagMigrationException("Migrate called, but no migration path from [%s] to [%s]"%(msg_from._type, msg_to._type))
  ^
  ./tools/rosbag/src/rosbag/rosbag_main.py:540:28: F821 undefined name 'raw_input'
  new_type = raw_input('>')
  ^
  ./tools/rosbag/src/rosbag/rosbag_main.py:544:32: F821 undefined name 'raw_input'
  new_type = raw_input('>')
  ^
  ./tools/rosbag/src/rosbag/rosbag_main.py:834:9: F821 undefined name 'parser'
  parser.error("Cannot find rosbag/encrypt executable")
  ^
  ./tools/rosgraph/src/rosgraph/names.py:63:30: F821 undefined name 'basestring'
  return isinstance(s, basestring) #Python 2.x
  ^
  ./tools/rosgraph/src/rosgraph/network.py:397:35: F821 undefined name 'unicode'
  str_cls = str if python3 else unicode
  ^
  ./tools/roslaunch/src/roslaunch/__init_\_.py:216:67: F821 undefined name 'f'
  parser.error("The following input files do not exist: %s"%f)
  ^
  ./tools/roslaunch/src/roslaunch/core.py:315:79: F821 undefined name 'msg'
  raise RLException("ERROR: master failed status check: %s"%msg)
  ^
  ./tools/roslaunch/src/roslaunch/server.py:262:103: F821 undefined name 'm'
  raise RLException("ERROR: roslaunch server URI is not a valid XML-RPC URI. Value is [%s]"%m.uri)
  ^
  ./tools/roslaunch/test/unit/test_roslaunch_pmon.py:82:31: F821 undefined name 'p'
  return self.procs.get(p, None)
  ^
  ./tools/rosmaster/src/rosmaster/main.py:139:5: F821 undefined name 'main'
  main()
  ^
  ./tools/rosmaster/src/rosmaster/master_api.py:547:100: F821 undefined name 's'
  _logger.warn('subscriber data stale (key [%s], listener [%s]): node API unknown'%(key, s))
  ^
  ./tools/rosmaster/src/rosmaster/validators.py:183:16: F821 undefined name 'is_global'
  if not is_global(param_value):
  ^
  ./tools/rosmaster/test/test_rosmaster_paramserver.py:308:101: F821 undefined name 'traceback'
  raise Exception("Exception raised while calling param_server.get_param(%s): %s"%(k, traceback.format_exc()))
  ^
  ./tools/rosmsg/src/rosmsg/__init_\_.py:181:64: F821 undefined name 'Time'
  if time_offset is not None and isinstance(val, Time):
  ^
  ./tools/rosparam/src/rosparam/__init_\_.py:354:134: F821 undefined name 'maxint'
  raise RosParamException("Overflow: Parameter Server integers must be 32-bit signed integers:\n\t-%s <= value <= %s"%(maxint - 1, maxint))
  ^
  ./tools/rosparam/src/rosparam/__init_\_.py:354:146: F821 undefined name 'maxint'
  raise RosParamException("Overflow: Parameter Server integers must be 32-bit signed integers:\n\t-%s <= value <= %s"%(maxint - 1, maxint))
  ^
  ./tools/rostest/src/rostest/__init_\_.py:211:17: F821 undefined name 'reload'
  reload(sys.modules[package])
  ^
  ./tools/rostopic/src/rostopic/__init_\_.py:285:70: F821 undefined name 'xrange'
  body = '\n'.join('   '.join(cols[h][i] for h in header) for i in xrange(n_rows))
  ^
  ./tools/topic_tools/test/test_mux_delete_add.py:71:17: E999 TabError: inconsistent use of tabs and spaces in indentation
  rospy.sleep(0.2)
  ^
  ./tools/topic_tools/test/test_mux_services.py:75:5: E999 TabError: inconsistent use of tabs and spaces in indentation
  try:
  ^
  ./utilities/message_filters/src/message_filters/__init_\_.py:220:18: F821 undefined name 'reduce'
  common = reduce(set.intersection, [set(q) for q in self.queues])
  ^
  ./utilities/roswtf/src/roswtf/graph.py:179:9: F821 undefined name 'rospy'
  rospy.Subscriber(t, msg_class)
  ^
  ./utilities/roswtf/src/roswtf/graph.py:179:29: F821 undefined name 'msg_class'
  rospy.Subscriber(t, msg_class)
  ^
  5     E999 SyntaxError: invalid syntax
  60    F821 undefined name 'basestring'
  65
  ```
  * import threading
  ```
  ./clients/rospy/src/rospy/impl/udpros.py:79:9: F821 undefined name 'threading'
  threading.start_new_thread(self.run, ())
  ^
  ```
* Contributors: Christopher Wecht, Dirk Thomas, Hans Gaiser, Markus Grimm, Martijn Buijs, Martin Pecka, Maxime St-Pierre, Paul Bovbel, Paweł Lorek, Victor Lamoine, Yong Li, Yuchen Ying, abencz, cclauss

1.14.3 (2018-08-06)
-------------------
* maintain exception info in RosOutHandler (`#1442 <https://github.com/ros/ros_comm/issues/1442>`_)

1.14.2 (2018-06-06)
-------------------
* fix some errors in some probably not frequented code paths (`#1415 <https://github.com/ros/ros_comm/issues/1415>`_)
* fix thread problem with get_topics() (`#1416 <https://github.com/ros/ros_comm/issues/1416>`_)

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* add API to suppress sequential identical messages (`#1309 <https://github.com/ros/ros_comm/issues/1309>`_)
* add parameter to stop clients from generating rosout topics list (`#1241 <https://github.com/ros/ros_comm/issues/1241>`_)
* add rosconsole echo (`#1324 <https://github.com/ros/ros_comm/issues/1324>`_)

1.13.6 (2018-02-05)
-------------------
* raise the correct exception from AnyMsg.serialize (`#1311 <https://github.com/ros/ros_comm/issues/1311>`_)
* remove unreachable exceptions (`#1260 <https://github.com/ros/ros_comm/issues/1260>`_)
* replace Thread.setDaemon() using new API (`#1276 <https://github.com/ros/ros_comm/issues/1276>`_)

1.13.5 (2017-11-09)
-------------------
* fix regresssion from 1.13.3 (`#1224 <https://github.com/ros/ros_comm/issues/1224>`_)

1.13.4 (2017-11-02)
-------------------
* fix uri in message (`#1213 <https://github.com/ros/ros_comm/issues/1213>`_, regression from 1.13.3)

1.13.3 (2017-10-25)
-------------------
* change rospy.Rate hz type from int to float (`#1177 <https://github.com/ros/ros_comm/issues/1177>`_)
* use defined error codes rather than hardcoded integers (`#1174 <https://github.com/ros/ros_comm/issues/1174>`_)
* improve log messages when waiting for service (`#1026 <https://github.com/ros/ros_comm/issues/1026>`_)
* improve logger tests (`#1144 <https://github.com/ros/ros_comm/issues/1144>`_)

1.13.2 (2017-08-15)
-------------------
* fix stack frame identification in rospy logging (`#1141 <https://github.com/ros/ros_comm/issues/1141>`_, regression from 1.13.1)

1.13.1 (2017-07-27)
-------------------
* improve rospy.logXXX_throttle performance (`#1091 <https://github.com/ros/ros_comm/pull/1091>`_)
* add option to reset timer when time moved backwards (`#1083 <https://github.com/ros/ros_comm/issues/1083>`_)
* abort topic lookup on connection refused (`#1044 <https://github.com/ros/ros_comm/pull/1044>`_)
* add rospy.logXXX_once (`#1041 <https://github.com/ros/ros_comm/issues/1041>`_)
* remove "ROS time moved backwards" log message (`#1027 <https://github.com/ros/ros_comm/pull/1027>`_)
* sleep in rospy wait_for_service even if exceptions raised (`#1025 <https://github.com/ros/ros_comm/pull/1025>`_)
* add named loggers (`#948 <https://github.com/ros/ros_comm/pull/948>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* make get_published_topics threadsafe (`#958 <https://github.com/ros/ros_comm/issues/958>`_)
* use poll in write_header() if available to support higher numbered fileno (`#929 <https://github.com/ros/ros_comm/pull/929>`_)
* use epoll instead of poll if available to gracefully close hung connections (`#831 <https://github.com/ros/ros_comm/issues/831>`_)
* fix Python 3 compatibility issues (`#565 <https://github.com/ros/ros_comm/issues/565>`_)

1.12.6 (2016-10-26)
-------------------
* improve reconnection logic on timeout and other common errors (`#851 <https://github.com/ros/ros_comm/pull/851>`_)
* remove duplicated function (`#783 <https://github.com/ros/ros_comm/pull/783>`_)

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* raise error on rospy.init_node with None or empty node name string (`#895 <https://github.com/ros/ros_comm/pull/895>`_)
* fix wrong type in docstring for rospy.Timer (`#878 <https://github.com/ros/ros_comm/pull/878>`_)
* fix order of init and publisher in example (`#873 <https://github.com/ros/ros_comm/pull/873>`_)

1.12.2 (2016-06-03)
-------------------
* add logXXX_throttle functions (`#812 <https://github.com/ros/ros_comm/pull/812>`_)

1.12.1 (2016-04-18)
-------------------

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------
* preserve identity of numpy_msg(T) (`#758 <https://github.com/ros/ros_comm/pull/758>`_)

1.11.16 (2015-11-09)
--------------------
* catch ROSInterruptException from rospy timers when shutting down (`#690 <https://github.com/ros/ros_comm/pull/690>`_)

1.11.15 (2015-10-13)
--------------------
* validate name after remapping (`#669 <https://github.com/ros/ros_comm/pull/669>`_)

1.11.14 (2015-09-19)
--------------------
* fix memory/thread leak with QueuedConnection (`#661 <https://github.com/ros/ros_comm/pull/661>`_)
* fix signaling already shutdown to client hooks with the appropriate signature (`#651 <https://github.com/ros/ros_comm/issues/651>`_)
* fix bug with missing current logger levels (`#631 <https://github.com/ros/ros_comm/pull/631>`_)

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------
* add rosconsole command line tool to change logger levels (`#576 <https://github.com/ros/ros_comm/pull/576>`_)
* add accessor for remaining time of the Rate class (`#588 <https://github.com/ros/ros_comm/pull/588>`_)
* fix high latency when using asynchronous publishing (`#547 <https://github.com/ros/ros_comm/issues/547>`_)
* fix error handling when publishing on Empty topic (`#566 <https://github.com/ros/ros_comm/pull/566>`_)

1.11.10 (2014-12-22)
--------------------
* add specific exception for time jumping backwards (`#485 <https://github.com/ros/ros_comm/issues/485>`_)
* make param functions thread-safe (`#523 <https://github.com/ros/ros_comm/pull/523>`_)
* fix infinitely retrying subscriber (`#533 <https://github.com/ros/ros_comm/issues/533>`_)
* fix removal of QueuedConnection leading to wrong subscriber count (`#526 <https://github.com/ros/ros_comm/issues/526>`_)
* fix TCPROS header validation when `callerid` header is not set (`#522 <https://github.com/ros/ros_comm/issues/522>`_, regression from 1.11.1)
* fix memory leak when using subcriber statistics (`#520 <https://github.com/ros/ros_comm/issues/520>`_)
* fix reported traffic in bytes from Python nodes (`#501 <https://github.com/ros/ros_comm/issues/501>`_)

1.11.9 (2014-08-18)
-------------------
* populate delivered_msgs field of TopicStatistics message (`#486 <https://github.com/ros/ros_comm/issues/486>`_)

1.11.8 (2014-08-04)
-------------------
* fix topic/connection statistics reporting code (`#482 <https://github.com/ros/ros_comm/issues/482>`_)

1.11.7 (2014-07-18)
-------------------

1.11.6 (2014-07-10)
-------------------
* make MasterProxy thread-safe (`#459 <https://github.com/ros/ros_comm/issues/459>`_)
* check ROS_HOSTNAME for localhost / ROS_IP for 127./::1 and prevent connections from other hosts in that case (`#452 <https://github.com/ros/ros_comm/issues/452>`)_

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_)

1.11.3 (2014-05-21)
-------------------
* allow shutdown hooks to be any callable object (`#410 <https://github.com/ros/ros_comm/issues/410>`_)
* add demux program and related scripts (`#407 <https://github.com/ros/ros_comm/issues/407>`_)
* add publisher queue_size to rostopic

1.11.2 (2014-05-08)
-------------------
* use publisher queue_size for statistics (`#398 <https://github.com/ros/ros_comm/issues/398>`_)

1.11.1 (2014-05-07)
-------------------
* improve asynchonous publishing performance (`#373 <https://github.com/ros/ros_comm/issues/373>`_)
* add warning when queue_size is omitted for rospy publisher (`#346 <https://github.com/ros/ros_comm/issues/346>`_)
* add optional topic/connection statistics (`#398 <https://github.com/ros/ros_comm/issues/398>`_)
* add transport information in SlaveAPI::getBusInfo() for roscpp & rospy (`#328 <https://github.com/ros/ros_comm/issues/328>`_)
* allow custom error handlers for services (`#375 <https://github.com/ros/ros_comm/issues/375>`_)
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* fix exception handling for queued connections (`#369 <https://github.com/ros/ros_comm/issues/369>`_)
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

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
* implement optional queueing for rospy publications (`#169 <https://github.com/ros/ros_comm/issues/169>`_)
* overwrite __repr__ for rospy.Duration and Time (`ros/genpy#24 <https://github.com/ros/genpy/issues/24>`_)
* add missing dependency on roscpp

1.9.50 (2013-10-04)
-------------------
* add support for python coverage tool to work in callbacks

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* make rospy nodes killable while waiting for master (`#262 <https://github.com/ros/ros_comm/issues/262>`_)

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------
* add missing run_depend on python-yaml
* allow configuration of ports for XML RPCs and TCP ROS
* fix race condition where rospy subscribers do not connect to all publisher
* fix closing and deregistering connection when connect fails (`#128 <https://github.com/ros/ros_comm/issues/128>`_)
* fix log level of RosOutHandler (`#210 <https://github.com/ros/ros_comm/issues/210>`_)

1.9.44 (2013-03-21)
-------------------

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* make dependencies on rospy optional by refactoring RosStreamHandler to rosgraph (`#179 <https://github.com/ros/ros_comm/issues/179>`_)

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add colorization for rospy log output (`#3691 <https://code.ros.org/trac/ros/ticket/3691>`_)
* fix socket polling under Windows (`#3959 <https://code.ros.org/trac/ros/ticket/3959>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
