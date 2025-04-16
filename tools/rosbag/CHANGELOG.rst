^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.11 (2021-04-06)
--------------------
* Handle SIGINT in rosbag play (`#2150 <https://github.com/ros/ros_comm/issues/2150>`_)
* Catch all exceptions in record thread (`#2151 <https://github.com/ros/ros_comm/issues/2151>`_)
* raw_input does not exist in python 3 (`#2143 <https://github.com/ros/ros_comm/issues/2143>`_)
* Contributors: Martin Pecka, Sebastian Scherer, pseyfert

1.15.10 (2021-03-18)
--------------------
* Add missing Boost (`#2108 <https://github.com/ros/ros_comm/issues/2108>`_)
* Start player in paused state (`#2086 <https://github.com/ros/ros_comm/issues/2086>`_)
* Contributors: Francisco Vina, Timo Röhling

1.15.9 (2020-10-16)
-------------------
* Update maintainers (`#2075 <https://github.com/ros/ros_comm/issues/2075>`_)
* Fix spelling (`#2066 <https://github.com/ros/ros_comm/issues/2066>`_)
* Gracefully stop recording upon SIGTERM and SIGINT (`#2038 <https://github.com/ros/ros_comm/issues/2038>`_)
* Fix compatibility issue with boost 1.73 and above (`#2023 <https://github.com/ros/ros_comm/issues/2023>`_)
* Use heapq.merge instead of custom merge sort code (`#2017 <https://github.com/ros/ros_comm/issues/2017>`_)
* Contributors: Devin Bonnie, Florian Friesdorf, Sean Yen, Shane Loretz, tomoya

1.15.8 (2020-07-23)
-------------------

1.15.7 (2020-05-28)
-------------------

1.15.6 (2020-05-21)
-------------------

1.15.5 (2020-05-15)
-------------------
* add option to repeat latched messages at the start of bag splits (`#1850 <https://github.com/ros/ros_comm/issues/1850>`_)
* fix bag migration failures caused by typo in connection_header assignment (`#1952 <https://github.com/ros/ros_comm/issues/1952>`_)

1.15.4 (2020-03-19)
-------------------
* restrict boost dependencies to components used (`#1871 <https://github.com/ros/ros_comm/issues/1871>`_)

1.15.3 (2020-02-28)
-------------------
* remove Boost version check since Noetic only targets platforms with 1.67+ (`#1903 <https://github.com/ros/ros_comm/issues/1903>`_)

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
* add quotes around file name so they can be click selected in terminal (`#1813 <https://github.com/ros/ros_comm/issues/1813>`_)
* catch exceptions by const ref (`#1874 <https://github.com/ros/ros_comm/issues/1874>`_)
* read GPG passphrase from an environment variable (`#1856 <https://github.com/ros/ros_comm/issues/1856>`_)
* fix missing import of roslib (`#1818 <https://github.com/ros/ros_comm/issues/1818>`_)
* fix regression from pycrypodome switchover (`#1814 <https://github.com/ros/ros_comm/issues/1814>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* add pycryptodome as default (`#1609 <https://github.com/ros/ros_comm/issues/1609>`_)
* encrypted rosbag fixes for Python 3 (`#1777 <https://github.com/ros/ros_comm/issues/1777>`_)
* fix bug in bag migration (`#1786 <https://github.com/ros/ros_comm/issues/1786>`_)
* keep latched topics latched (`#1708 <https://github.com/ros/ros_comm/issues/1708>`_)
* wrap the rosbag filter eval in a lambda (`#1712 <https://github.com/ros/ros_comm/issues/1712>`_)
* record: fix signed int overflow (`#1741 <https://github.com/ros/ros_comm/issues/1741>`_)
* switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (`#1688 <https://github.com/ros/ros_comm/issues/1688>`_)
* pickleable rosbag exceptions (`#1210 <https://github.com/ros/ros_comm/issues/1210>`_ revisited). (`#1652 <https://github.com/ros/ros_comm/issues/1652>`_)
* fix topic message count for rosbag indexed v1.2 (`#1648 <https://github.com/ros/ros_comm/issues/1648>`_)
* fix wrong error handling in migration (`#1639 <https://github.com/ros/ros_comm/issues/1639>`_)
* modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated (`#1641 <https://github.com/ros/ros_comm/issues/1641>`_)
* fix IOError during Python file operation (`#1617 <https://github.com/ros/ros_comm/issues/1617>`_)
* add Windows.h usage explicitly (`#44 <https://github.com/ros/ros_comm/issues/44>`_) (`#1616 <https://github.com/ros/ros_comm/issues/1616>`_)
* fix waitForSubscribers hanging with simtime (`#1543 <https://github.com/ros/ros_comm/issues/1543>`_)
* publish last message from latch topics when start time > 0 (`#1537 <https://github.com/ros/ros_comm/issues/1537>`_)
* add a new option to publish when a bag write begin (`#1527 <https://github.com/ros/ros_comm/issues/1527>`_)

1.16.0 (2022-02-23)
-------------------
* Fixing latched record updating (#28)
* Writing out connection header for latched topics (#26)
* Passing time args through to the actual rosbag play executable
* Adding real clock time display option (#25)
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* Fix latched timestamps (#15)
  * Overwrite message receipt time to prevent gaps.
  * Revert unrelated whitespace changes
* Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
* 1.15.11
* handle SIGINT in rosbag play (#2150)
  This is copy&paste from https://github.com/ros/ros_comm/pull/2038/files#diff-8c5ae1a482044f103bf9b2fa9188ff9639ed617f259a8dd816075bc9e4005ef9R152
  where the SIGINT handler was added to rosbag record.
* rosbag: recorder: Catch all exceptions in record thread. (#2151)
* fix issue 2141: raw_input does not exist in python 3 (#2143)
  * fix for python 3.x where raw_input does not exist anymore
* 1.15.10
* Add missing Boost (#2108)
* rosbag: start player in paused state (#2086)
* 1.15.9 package.xmls
* 1.15.9
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
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* Use heapq.merge instead of custom merge sort code (#2017)
* 1.15.8
* update changelogs
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Add option to repeat latched messages at the start of bag splits (#1850)
  * Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
  * Overwrite message receipt time to prevent gaps.
  (cherry picked from commit e407e164d6f69c6dea17cb59ca07fef4629e26aa)
  * Revert unrelated whitespace changes
  (cherry picked from commit 836a4e5f1338697ecb643790e400b7068555f569)
  * revert unrelated whitespace changes
  * revert unrelated whitespace changes
  Co-authored-by: eric <etappan@locusrobotics.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix bag migration failures caused by typo in connection_header assignment (#1952)
  Bag migration failed with follwoing errors:
  NameError: global name 'connection_header' is not defined
  This change fixes that by using local name conn_header
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
* Add quotes around file name so they can be click selected in terminal. (#1813)
  It's convenient to click on the printed rosbag name to copy it and use as a parameter elsewhere, but the period will get included in the click without the quotes (at least in Ubuntu 18.04 + gnome terminal).
* [rosbag] Catch exceptions by const ref. (#1874)
* Read GPG passphrase from an environment variable (#1856)
  * Read passphrase from an environment variable
  * Accept passphrase from the encryptor's initialize method
* Bug: roslib not being imported (#1818)
  * Added import of roslib for bug fix.
  * Changed import to more specific import.
* Correct an issue from pycrypodome switchover. (#1814)
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Add pycryptodome as default on melodic (#1609)
* Encrypted rosbag fixes for Python 3. (#1777)
* fixed #1776 deadcode (#1786)
* rosbag fix: keep latched topics latched (#1708)
* Wrap the rosbag filter eval in a lambda (#1712)
* rosbag/record: fix signed int overflow (#1741)
* Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (#1688)
  * Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning
  * Change all usages of yaml.load to yaml.safe_load
  * Extend PyYAML's SafeLoader and use it with `yaml.load`
  Also added convenience functions for using this loader for reuse in
  `roslaunch`
  * fix typo in rosparam.yaml_load_all
  * Modify Loader and SafeLoader in yaml module directly
  * Revert whitespace change
  * Revert unrelated change to import through global variable construction
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* fix topic message count for rosbag indexed v1.2. (#1648)
* Fix wrong error handling in migration. (#1639)
  * Fix wrong error handling in migration.
  self.(old|new)_types is supposed to be a dictionary, not a list
  * Address PR 1639 comments.
  * remove duplicate warning message
* rosbag modernization: replaced BOOST_FOREACH with range-based for-loo… (#1641)
  * rosbag modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated
  * /rosbag: changed formatting
  * single level indentation
  * single level indentation
* fix IOError during Python file operation (#1617)
  * fix IOError from python file operation in r+ mode (#41)
  * move file.seek into _stop_writing, add comment (#42)
* add Windows.h usage explicitly (#44) (#1616)
  * attempt to remove unused header.
  * Add windows.h usage explicitly
  * Update statistics.h
* [rosbag] Fix waitForSubscribers hanging with simtime (#1543)
  * [rosbag] Fix waitForSubscribers hanging with simtime
  Use Walltime for sleep as clock is not running yet
  * Simplify wallsleep in waitForSubscribers
  Co-Authored-By: AlexReimann <alexander.reimann@enway.ai>
* Publish last message from latch topics when start time > 0 (#1537)
  * Publish last message from latch topics when start time > 0
  * cuddle brace
  I know the ROS 1 style guide says otherwise but for my sanity of reading it I saved the vertical space
* Add a new option to publish when a bag write begin (#1527)
  * Add a new option to publish when a bag write begin
  * Add the option in rosbag_main to use rosbag record
* Contributors: Alexander Reimann, AnthonyBirot, Christopher Wecht, Dallin Briggs, Daniel Wang, Devin Bonnie, Dirk Thomas, Enrique Fernández Perdomo, Eric Tappan, Florian Friesdorf, Francisco Vina, Gary Servin, Jacob Perron, James Xu, Lucas Walter, Martijn Buijs, Martin Pecka, Maxime St-Pierre, Mikael Arguedas, Mike Purvis, Natesh Narain, Olivier Mangin, Sean Yen, Sebastian Scherer, Shane Loretz, Thomas, Timo Röhling, Tom Moore, kmiku7, pseyfert, tomoya

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* 1.16.0
* Update changelogs
* Fixing latched record updating (#28)
* Writing out connection header for latched topics (#26)
* Passing time args through to the actual rosbag play executable
* Adding real clock time display option (#25)
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* Fix latched timestamps (#15)
  * Overwrite message receipt time to prevent gaps.
  * Revert unrelated whitespace changes
* Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
* 1.15.11
* handle SIGINT in rosbag play (#2150)
  This is copy&paste from https://github.com/ros/ros_comm/pull/2038/files#diff-8c5ae1a482044f103bf9b2fa9188ff9639ed617f259a8dd816075bc9e4005ef9R152
  where the SIGINT handler was added to rosbag record.
* rosbag: recorder: Catch all exceptions in record thread. (#2151)
* fix issue 2141: raw_input does not exist in python 3 (#2143)
  * fix for python 3.x where raw_input does not exist anymore
* 1.15.10
* Add missing Boost (#2108)
* rosbag: start player in paused state (#2086)
* 1.15.9 package.xmls
* 1.15.9
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
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* Use heapq.merge instead of custom merge sort code (#2017)
* 1.15.8
* update changelogs
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Add option to repeat latched messages at the start of bag splits (#1850)
  * Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
  * Overwrite message receipt time to prevent gaps.
  (cherry picked from commit e407e164d6f69c6dea17cb59ca07fef4629e26aa)
  * Revert unrelated whitespace changes
  (cherry picked from commit 836a4e5f1338697ecb643790e400b7068555f569)
  * revert unrelated whitespace changes
  * revert unrelated whitespace changes
  Co-authored-by: eric <etappan@locusrobotics.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix bag migration failures caused by typo in connection_header assignment (#1952)
  Bag migration failed with follwoing errors:
  NameError: global name 'connection_header' is not defined
  This change fixes that by using local name conn_header
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
* Add quotes around file name so they can be click selected in terminal. (#1813)
  It's convenient to click on the printed rosbag name to copy it and use as a parameter elsewhere, but the period will get included in the click without the quotes (at least in Ubuntu 18.04 + gnome terminal).
* [rosbag] Catch exceptions by const ref. (#1874)
* Read GPG passphrase from an environment variable (#1856)
  * Read passphrase from an environment variable
  * Accept passphrase from the encryptor's initialize method
* Bug: roslib not being imported (#1818)
  * Added import of roslib for bug fix.
  * Changed import to more specific import.
* Correct an issue from pycrypodome switchover. (#1814)
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Add pycryptodome as default on melodic (#1609)
* Encrypted rosbag fixes for Python 3. (#1777)
* fixed #1776 deadcode (#1786)
* rosbag fix: keep latched topics latched (#1708)
* Wrap the rosbag filter eval in a lambda (#1712)
* rosbag/record: fix signed int overflow (#1741)
* Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (#1688)
  * Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning
  * Change all usages of yaml.load to yaml.safe_load
  * Extend PyYAML's SafeLoader and use it with `yaml.load`
  Also added convenience functions for using this loader for reuse in
  `roslaunch`
  * fix typo in rosparam.yaml_load_all
  * Modify Loader and SafeLoader in yaml module directly
  * Revert whitespace change
  * Revert unrelated change to import through global variable construction
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* fix topic message count for rosbag indexed v1.2. (#1648)
* Fix wrong error handling in migration. (#1639)
  * Fix wrong error handling in migration.
  self.(old|new)_types is supposed to be a dictionary, not a list
  * Address PR 1639 comments.
  * remove duplicate warning message
* rosbag modernization: replaced BOOST_FOREACH with range-based for-loo… (#1641)
  * rosbag modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated
  * /rosbag: changed formatting
  * single level indentation
  * single level indentation
* fix IOError during Python file operation (#1617)
  * fix IOError from python file operation in r+ mode (#41)
  * move file.seek into _stop_writing, add comment (#42)
* add Windows.h usage explicitly (#44) (#1616)
  * attempt to remove unused header.
  * Add windows.h usage explicitly
  * Update statistics.h
* [rosbag] Fix waitForSubscribers hanging with simtime (#1543)
  * [rosbag] Fix waitForSubscribers hanging with simtime
  Use Walltime for sleep as clock is not running yet
  * Simplify wallsleep in waitForSubscribers
  Co-Authored-By: AlexReimann <alexander.reimann@enway.ai>
* Publish last message from latch topics when start time > 0 (#1537)
  * Publish last message from latch topics when start time > 0
  * cuddle brace
  I know the ROS 1 style guide says otherwise but for my sanity of reading it I saved the vertical space
* Add a new option to publish when a bag write begin (#1527)
  * Add a new option to publish when a bag write begin
  * Add the option in rosbag_main to use rosbag record
* Contributors: Alexander Reimann, AnthonyBirot, Christopher Wecht, Dallin Briggs, Daniel Wang, Devin Bonnie, Dirk Thomas, Enrique Fernández Perdomo, Eric Tappan, Florian Friesdorf, Francisco Vina, Gary Servin, Jacob Perron, James Xu, Lucas Walter, Martijn Buijs, Martin Pecka, Maxime St-Pierre, Mikael Arguedas, Mike Purvis, Natesh Narain, Olivier Mangin, Sean Yen, Sebastian Scherer, Shane Loretz, Thomas, Timo Röhling, Tom Moore, kmiku7, pseyfert, tomoya

1.19.0 (2023-09-25)
-------------------
* Added playback finished status (#32)
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* Fixing latched record updating (#28)
* Writing out connection header for latched topics (#26)
* Passing time args through to the actual rosbag play executable
* Adding real clock time display option (#25)
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* Fix latched timestamps (#15)
  * Overwrite message receipt time to prevent gaps.
  * Revert unrelated whitespace changes
* Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
* 1.15.11
* handle SIGINT in rosbag play (#2150)
  This is copy&paste from https://github.com/ros/ros_comm/pull/2038/files#diff-8c5ae1a482044f103bf9b2fa9188ff9639ed617f259a8dd816075bc9e4005ef9R152
  where the SIGINT handler was added to rosbag record.
* rosbag: recorder: Catch all exceptions in record thread. (#2151)
* fix issue 2141: raw_input does not exist in python 3 (#2143)
  * fix for python 3.x where raw_input does not exist anymore
* 1.15.10
* Add missing Boost (#2108)
* rosbag: start player in paused state (#2086)
* 1.15.9 package.xmls
* 1.15.9
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
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* Use heapq.merge instead of custom merge sort code (#2017)
* 1.15.8
* update changelogs
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Add option to repeat latched messages at the start of bag splits (#1850)
  * Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
  * Overwrite message receipt time to prevent gaps.
  (cherry picked from commit e407e164d6f69c6dea17cb59ca07fef4629e26aa)
  * Revert unrelated whitespace changes
  (cherry picked from commit 836a4e5f1338697ecb643790e400b7068555f569)
  * revert unrelated whitespace changes
  * revert unrelated whitespace changes
  Co-authored-by: eric <etappan@locusrobotics.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix bag migration failures caused by typo in connection_header assignment (#1952)
  Bag migration failed with follwoing errors:
  NameError: global name 'connection_header' is not defined
  This change fixes that by using local name conn_header
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
* Add quotes around file name so they can be click selected in terminal. (#1813)
  It's convenient to click on the printed rosbag name to copy it and use as a parameter elsewhere, but the period will get included in the click without the quotes (at least in Ubuntu 18.04 + gnome terminal).
* [rosbag] Catch exceptions by const ref. (#1874)
* Read GPG passphrase from an environment variable (#1856)
  * Read passphrase from an environment variable
  * Accept passphrase from the encryptor's initialize method
* Bug: roslib not being imported (#1818)
  * Added import of roslib for bug fix.
  * Changed import to more specific import.
* Correct an issue from pycrypodome switchover. (#1814)
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Add pycryptodome as default on melodic (#1609)
* Encrypted rosbag fixes for Python 3. (#1777)
* fixed #1776 deadcode (#1786)
* rosbag fix: keep latched topics latched (#1708)
* Wrap the rosbag filter eval in a lambda (#1712)
* rosbag/record: fix signed int overflow (#1741)
* Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (#1688)
  * Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning
  * Change all usages of yaml.load to yaml.safe_load
  * Extend PyYAML's SafeLoader and use it with `yaml.load`
  Also added convenience functions for using this loader for reuse in
  `roslaunch`
  * fix typo in rosparam.yaml_load_all
  * Modify Loader and SafeLoader in yaml module directly
  * Revert whitespace change
  * Revert unrelated change to import through global variable construction
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* fix topic message count for rosbag indexed v1.2. (#1648)
* Fix wrong error handling in migration. (#1639)
  * Fix wrong error handling in migration.
  self.(old|new)_types is supposed to be a dictionary, not a list
  * Address PR 1639 comments.
  * remove duplicate warning message
* rosbag modernization: replaced BOOST_FOREACH with range-based for-loo… (#1641)
  * rosbag modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated
  * /rosbag: changed formatting
  * single level indentation
  * single level indentation
* fix IOError during Python file operation (#1617)
  * fix IOError from python file operation in r+ mode (#41)
  * move file.seek into _stop_writing, add comment (#42)
* add Windows.h usage explicitly (#44) (#1616)
  * attempt to remove unused header.
  * Add windows.h usage explicitly
  * Update statistics.h
* [rosbag] Fix waitForSubscribers hanging with simtime (#1543)
  * [rosbag] Fix waitForSubscribers hanging with simtime
  Use Walltime for sleep as clock is not running yet
  * Simplify wallsleep in waitForSubscribers
  Co-Authored-By: AlexReimann <alexander.reimann@enway.ai>
* Publish last message from latch topics when start time > 0 (#1537)
  * Publish last message from latch topics when start time > 0
  * cuddle brace
  I know the ROS 1 style guide says otherwise but for my sanity of reading it I saved the vertical space
* Add a new option to publish when a bag write begin (#1527)
  * Add a new option to publish when a bag write begin
  * Add the option in rosbag_main to use rosbag record
* Contributors: Alexander Reimann, AnthonyBirot, Carlos Mendes, Christopher Wecht, Dallin Briggs, Daniel Wang, Devin Bonnie, Dirk Thomas, Enrique Fernández Perdomo, Eric Tappan, Florian Friesdorf, Francisco Vina, Gary Servin, Jacob Perron, James Xu, Lucas Walter, Martijn Buijs, Martin Pecka, Maxime St-Pierre, Mikael Arguedas, Mike Purvis, Natesh Narain, Olivier Mangin, Sean Yen, Sebastian Scherer, Shane Loretz, Thomas, Timo Röhling, Tom Moore, kmiku7, pseyfert, tomoya

1.23.1 (2025-04-16)
-------------------

1.23.0 (2025-02-04)
-------------------
* Copy latched messages once per split with connection header (#37)
* Contributors: nleblanc-lr

1.22.0 (2024-09-16)
-------------------

1.21.0 (2024-06-17)
-------------------

1.20.0 (2024-02-02)
-------------------
* 1.19.0
* Update changelogs
* Added playback finished status (#32)
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* Fixing latched record updating (#28)
* Writing out connection header for latched topics (#26)
* Passing time args through to the actual rosbag play executable
* Adding real clock time display option (#25)
* REMOVEME: Force to use python3 for now
* Initializing the repeat_latched option (#17)
  * Initializing the repeat_latched option and adding test
* Fix latched timestamps (#15)
  * Overwrite message receipt time to prevent gaps.
  * Revert unrelated whitespace changes
* Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
* 1.15.11
* handle SIGINT in rosbag play (#2150)
  This is copy&paste from https://github.com/ros/ros_comm/pull/2038/files#diff-8c5ae1a482044f103bf9b2fa9188ff9639ed617f259a8dd816075bc9e4005ef9R152
  where the SIGINT handler was added to rosbag record.
* rosbag: recorder: Catch all exceptions in record thread. (#2151)
* fix issue 2141: raw_input does not exist in python 3 (#2143)
  * fix for python 3.x where raw_input does not exist anymore
* 1.15.10
* Add missing Boost (#2108)
* rosbag: start player in paused state (#2086)
* 1.15.9 package.xmls
* 1.15.9
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
* [roscpp] Update boost::placeholders usage for boost 1.73 (and later) (#2023)
  * more port fix.
  * boost::placeholders migration.
  * fix more.
  * revert boost/bind/bind.hpp for back-compatible.
* Use heapq.merge instead of custom merge sort code (#2017)
* 1.15.8
* update changelogs
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Add option to repeat latched messages at the start of bag splits (#1850)
  * Added --repeat-latched option to rosbag record
  (cherry picked from commit f777d6e39ec776cafde481f3462e80ff230156a9)
  * Overwrite message receipt time to prevent gaps.
  (cherry picked from commit e407e164d6f69c6dea17cb59ca07fef4629e26aa)
  * Revert unrelated whitespace changes
  (cherry picked from commit 836a4e5f1338697ecb643790e400b7068555f569)
  * revert unrelated whitespace changes
  * revert unrelated whitespace changes
  Co-authored-by: eric <etappan@locusrobotics.com>
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Fix bag migration failures caused by typo in connection_header assignment (#1952)
  Bag migration failed with follwoing errors:
  NameError: global name 'connection_header' is not defined
  This change fixes that by using local name conn_header
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
* Add quotes around file name so they can be click selected in terminal. (#1813)
  It's convenient to click on the printed rosbag name to copy it and use as a parameter elsewhere, but the period will get included in the click without the quotes (at least in Ubuntu 18.04 + gnome terminal).
* [rosbag] Catch exceptions by const ref. (#1874)
* Read GPG passphrase from an environment variable (#1856)
  * Read passphrase from an environment variable
  * Accept passphrase from the encryptor's initialize method
* Bug: roslib not being imported (#1818)
  * Added import of roslib for bug fix.
  * Changed import to more specific import.
* Correct an issue from pycrypodome switchover. (#1814)
* use condition attributes to specify Python 2 and 3 dependencies (#1792)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
* Add pycryptodome as default on melodic (#1609)
* Encrypted rosbag fixes for Python 3. (#1777)
* fixed #1776 deadcode (#1786)
* rosbag fix: keep latched topics latched (#1708)
* Wrap the rosbag filter eval in a lambda (#1712)
* rosbag/record: fix signed int overflow (#1741)
* Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (#1688)
  * Switch to yaml.safe_load(_all) to prevent YAMLLoadWarning
  * Change all usages of yaml.load to yaml.safe_load
  * Extend PyYAML's SafeLoader and use it with `yaml.load`
  Also added convenience functions for using this loader for reuse in
  `roslaunch`
  * fix typo in rosparam.yaml_load_all
  * Modify Loader and SafeLoader in yaml module directly
  * Revert whitespace change
  * Revert unrelated change to import through global variable construction
* Pickleable rosbag exceptions (#1210 revisited). (#1652)
  * test_rosbag/test_bag.py: test, if rosbag exception can be pickled
  * rosbag/bag.py: rosbag exceptions can now be unpickled
  * pep8
* fix topic message count for rosbag indexed v1.2. (#1648)
* Fix wrong error handling in migration. (#1639)
  * Fix wrong error handling in migration.
  self.(old|new)_types is supposed to be a dictionary, not a list
  * Address PR 1639 comments.
  * remove duplicate warning message
* rosbag modernization: replaced BOOST_FOREACH with range-based for-loo… (#1641)
  * rosbag modernization: replaced BOOST_FOREACH with range-based for-loops, used algorithm where appropriated
  * /rosbag: changed formatting
  * single level indentation
  * single level indentation
* fix IOError during Python file operation (#1617)
  * fix IOError from python file operation in r+ mode (#41)
  * move file.seek into _stop_writing, add comment (#42)
* add Windows.h usage explicitly (#44) (#1616)
  * attempt to remove unused header.
  * Add windows.h usage explicitly
  * Update statistics.h
* [rosbag] Fix waitForSubscribers hanging with simtime (#1543)
  * [rosbag] Fix waitForSubscribers hanging with simtime
  Use Walltime for sleep as clock is not running yet
  * Simplify wallsleep in waitForSubscribers
  Co-Authored-By: AlexReimann <alexander.reimann@enway.ai>
* Publish last message from latch topics when start time > 0 (#1537)
  * Publish last message from latch topics when start time > 0
  * cuddle brace
  I know the ROS 1 style guide says otherwise but for my sanity of reading it I saved the vertical space
* Add a new option to publish when a bag write begin (#1527)
  * Add a new option to publish when a bag write begin
  * Add the option in rosbag_main to use rosbag record
* Contributors: Alexander Reimann, AnthonyBirot, Carlos Mendes, Christopher Wecht, Dallin Briggs, Daniel Wang, Devin Bonnie, Dirk Thomas, Enrique Fernández Perdomo, Eric Tappan, Florian Friesdorf, Francisco Vina, Gary Servin, Jacob Perron, James Xu, Lucas Walter, Martijn Buijs, Martin Pecka, Maxime St-Pierre, Mikael Arguedas, Mike Purvis, Natesh Narain, Olivier Mangin, Sean Yen, Sebastian Scherer, Shane Loretz, Thomas, Timo Röhling, Tom Moore, kmiku7, pseyfert, tomoya

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
