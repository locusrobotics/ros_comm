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

1.22.1 (2024-10-01)
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
