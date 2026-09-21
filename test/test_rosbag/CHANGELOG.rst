^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2022-02-23)
-------------------
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* fix misspell. (#2066)
* Gracefully stop recording upon SIGTERM and SIGINT (#2038)
  * Add SIGTERM and SIGINT handlers to rosbag record
  * Add unit test for rosbag record SIGINT handling
  Add unit test for rosbag record SIGTERM handling
  * Address review comments
  Fix sending SIGINT to main process
  * Revert added whitespace
  * Revert SIGINT handler addition: use default
  * Remove unnecessary wait
  * Use BSD License
  * Add test improvements
  * Move test helper function
  * Remove redundant test rosbag launch
  * Add Amazon to new python test copyright
  * Remove unrelated whitespace
  * Split record cleanup unit tests
  Add record cleanup unit test helper
  * Revert whitespace change
  * revert white space change
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* catch polymorphic exceptions by reference (#1887)
  * catch polymorphic exceptions by reference
  * make the catched exception references const
* Bump CMake version to avoid CMP0048 warning (#1869)
* test_rosbag: add target dependency to fix unit test failures in parallel builds (#1877)
  See https://github.com/ros/ros_comm/pull/1651#issuecomment-482148146 for details.
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* more Python 3 compatibility (#1784)
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* rostest: fix flaky hztests (#1661)
  * rostest: fix flaky hztests
  * add retry to all hztests
  * fix concerns
  * fix more wrong retry-attributes
* test_rosbag modernization: replaced BOOST_FOREACH with range-based for-loops (#1642)
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Christopher Wecht, Devin Bonnie, Dirk Thomas, Gary Servin, Jacob Perron, Mikael Arguedas, Shane Loretz, Tom Moore, beetleskin, tomoya

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* fix misspell. (#2066)
* Gracefully stop recording upon SIGTERM and SIGINT (#2038)
  * Add SIGTERM and SIGINT handlers to rosbag record
  * Add unit test for rosbag record SIGINT handling
  Add unit test for rosbag record SIGTERM handling
  * Address review comments
  Fix sending SIGINT to main process
  * Revert added whitespace
  * Revert SIGINT handler addition: use default
  * Remove unnecessary wait
  * Use BSD License
  * Add test improvements
  * Move test helper function
  * Remove redundant test rosbag launch
  * Add Amazon to new python test copyright
  * Remove unrelated whitespace
  * Split record cleanup unit tests
  Add record cleanup unit test helper
  * Revert whitespace change
  * revert white space change
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* catch polymorphic exceptions by reference (#1887)
  * catch polymorphic exceptions by reference
  * make the catched exception references const
* Bump CMake version to avoid CMP0048 warning (#1869)
* test_rosbag: add target dependency to fix unit test failures in parallel builds (#1877)
  See https://github.com/ros/ros_comm/pull/1651#issuecomment-482148146 for details.
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* more Python 3 compatibility (#1784)
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* rostest: fix flaky hztests (#1661)
  * rostest: fix flaky hztests
  * add retry to all hztests
  * fix concerns
  * fix more wrong retry-attributes
* test_rosbag modernization: replaced BOOST_FOREACH with range-based for-loops (#1642)
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Christopher Wecht, Devin Bonnie, Dirk Thomas, Gary Servin, Jacob Perron, Mikael Arguedas, Shane Loretz, Tom Moore, beetleskin, tomoya

1.19.0 (2023-09-25)
-------------------
* 1.18.0
* Update changelogs
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* fix misspell. (#2066)
* Gracefully stop recording upon SIGTERM and SIGINT (#2038)
  * Add SIGTERM and SIGINT handlers to rosbag record
  * Add unit test for rosbag record SIGINT handling
  Add unit test for rosbag record SIGTERM handling
  * Address review comments
  Fix sending SIGINT to main process
  * Revert added whitespace
  * Revert SIGINT handler addition: use default
  * Remove unnecessary wait
  * Use BSD License
  * Add test improvements
  * Move test helper function
  * Remove redundant test rosbag launch
  * Add Amazon to new python test copyright
  * Remove unrelated whitespace
  * Split record cleanup unit tests
  Add record cleanup unit test helper
  * Revert whitespace change
  * revert white space change
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* catch polymorphic exceptions by reference (#1887)
  * catch polymorphic exceptions by reference
  * make the catched exception references const
* Bump CMake version to avoid CMP0048 warning (#1869)
* test_rosbag: add target dependency to fix unit test failures in parallel builds (#1877)
  See https://github.com/ros/ros_comm/pull/1651#issuecomment-482148146 for details.
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* more Python 3 compatibility (#1784)
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* rostest: fix flaky hztests (#1661)
  * rostest: fix flaky hztests
  * add retry to all hztests
  * fix concerns
  * fix more wrong retry-attributes
* test_rosbag modernization: replaced BOOST_FOREACH with range-based for-loops (#1642)
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Christopher Wecht, Devin Bonnie, Dirk Thomas, Gary Servin, Jacob Perron, Mikael Arguedas, Shane Loretz, Tom Moore, beetleskin, tomoya

Forthcoming
-----------
* RST-13777 roscore cpp: master [26.1.0] (#59)
* RST-13777 Fixing ros_comm tests (#54)
  * Fixing ros_comm tests
* Fixes for Python 3.12
  With fixes by Jochen Sprickerhof.
  Taken from
  https://salsa.debian.org/science-team/ros-ros-comm/-/blob/b74ca5c2c868a084ab36e46d68f7775518ac4c58/debian/patches/0016-Fixes-for-Python-3.12.patch
  (cherry picked from commit 42cd22e509907d1e89765f8d1a27cbb201321d28)
* 1.17.0
* 1.16.0
* 1.15.15
* Move @jacobperron from maintainer to author (#2302)
* rosbag reindex bugfix - seek to truncated position after broken chunk (#2286)
  The truncate operation left the current `f.tell()` read head at the pre-truncated position, so the chunk infos that are written on closing the file started writing at this pre-truncated position, leaving dangling broken chunk data in between the last good chunk and the first file-end chunkinfo.
* 1.15.14
* 1.15.13
* 1.15.12
* Contributors: Emerson Knapp, Jacob Perron, Matthias Klose, Michael Carroll, Shane Loretz, Tom Moore

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
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* fix misspell. (#2066)
* Gracefully stop recording upon SIGTERM and SIGINT (#2038)
  * Add SIGTERM and SIGINT handlers to rosbag record
  * Add unit test for rosbag record SIGINT handling
  Add unit test for rosbag record SIGTERM handling
  * Address review comments
  Fix sending SIGINT to main process
  * Revert added whitespace
  * Revert SIGINT handler addition: use default
  * Remove unnecessary wait
  * Use BSD License
  * Add test improvements
  * Move test helper function
  * Remove redundant test rosbag launch
  * Add Amazon to new python test copyright
  * Remove unrelated whitespace
  * Split record cleanup unit tests
  Add record cleanup unit test helper
  * Revert whitespace change
  * revert white space change
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* [noetic] Restrict boost dependencies to components used (#1871)
  * [roscpp] declare specific boost dependencies
  * [rosbag] declare specific boost dependencies
  * [rosbag_storage] declare specific boost dependencies
  * [rostest] declare specific boost dependencies
  * [xmlrpcpp] declare specific boost dependencies
  * [message_filters] declare specific boost dependencies
  * [test_rosbag] declare specific boost dependencies
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* catch polymorphic exceptions by reference (#1887)
  * catch polymorphic exceptions by reference
  * make the catched exception references const
* Bump CMake version to avoid CMP0048 warning (#1869)
* test_rosbag: add target dependency to fix unit test failures in parallel builds (#1877)
  See https://github.com/ros/ros_comm/pull/1651#issuecomment-482148146 for details.
  Co-authored-by: Johannes Meyer <johannes@intermodalics.eu>
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* more Python 3 compatibility (#1784)
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* rostest: fix flaky hztests (#1661)
  * rostest: fix flaky hztests
  * add retry to all hztests
  * fix concerns
  * fix more wrong retry-attributes
* test_rosbag modernization: replaced BOOST_FOREACH with range-based for-loops (#1642)
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Christopher Wecht, Devin Bonnie, Dirk Thomas, Gary Servin, Jacob Perron, Mikael Arguedas, Shane Loretz, Tom Moore, beetleskin, tomoya

1.9.0 (2022-02-23)
-------------------
