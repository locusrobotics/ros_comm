^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslaunch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.11 (2021-04-06)
--------------------

1.15.10 (2021-03-18)
--------------------
* Fix AttributeError isAlive (`#2092 <https://github.com/ros/ros_comm/issues/2092>`_)
* Contributors: Brutus The Tschiepel

1.15.9 (2020-10-16)
-------------------
* Update maintainers (`#2075 <https://github.com/ros/ros_comm/issues/2075>`_)
* Fix handling of single quotes in command arguments on Windows (`#2051 <https://github.com/ros/ros_comm/issues/2051>`_)
* Fix spelling (`#2066 <https://github.com/ros/ros_comm/issues/2066>`_)
* Clearer error message (`#2035 <https://github.com/ros/ros_comm/issues/2035>`_)
* Ignore underscores when parsing literal numeric values for Python 3 compatibility (`#2022 <https://github.com/ros/ros_comm/issues/2022>`_)
* Contributors: Andreas Vinter-Hviid, Jochen Sprickerhof, Sean Yen, Shane Loretz, tomoya

1.15.8 (2020-07-23)
-------------------
* fix bad relative import (still Python 2 style) (`#1973 <https://github.com/ros/ros_comm/issues/1973>`_)

1.15.7 (2020-05-28)
-------------------
* fix NameError in launch error handling (`#1965 <https://github.com/ros/ros_comm/issues/1965>`_)

1.15.6 (2020-05-21)
-------------------

1.15.5 (2020-05-15)
-------------------
* add --sigint-timeout and --sigterm-timeout parameters (`#1937 <https://github.com/ros/ros_comm/issues/1937>`_)
* roslaunch-check: search dir recursively (`#1914 <https://github.com/ros/ros_comm/issues/1914>`_)
* sort printed nodes by namespace alphabetically (`#1934 <https://github.com/ros/ros_comm/issues/1934>`_)
* remove pycrypto import (not used) (`#1922 <https://github.com/ros/ros_comm/issues/1922>`_)

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
* update test to pass with old and new yaml (`#1893 <https://github.com/ros/ros_comm/issues/1893>`_)

1.14.4 (2020-02-20)
-------------------
* allow empty machine arg in node tag (`#1885 <https://github.com/ros/ros_comm/issues/1885>`_)
* use double quotes for portable roslaunch-check command (`#1883 <https://github.com/ros/ros_comm/issues/1883>`_)
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* wrap env prefix with double quotes (`#1810 <https://github.com/ros/ros_comm/issues/1810>`_)
* add ignore default args option to roslaunch-check (`#1788 <https://github.com/ros/ros_comm/issues/1788>`_)
* add platform check for --args code path (`#1809 <https://github.com/ros/ros_comm/issues/1809>`_)
* [Windows] workaround for Python 2 xmlrpc performance issues (`#1872 <https://github.com/ros/ros_comm/issues/1872>`_)
* allow empty ssh password for remote launching (`#1826 <https://github.com/ros/ros_comm/issues/1826>`_)
* [Windows] escape drive as well as path separator (`#1815 <https://github.com/ros/ros_comm/issues/1815>`_)
* more Python 3 compatibility (`#1795 <https://github.com/ros/ros_comm/issues/1795>`_)
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/ros/ros_comm/issues/1792>`_)
* [Windows] skip `cat` related test cases on Windows build (`#1724 <https://github.com/ros/ros_comm/issues/1724>`_)
* [Windows] use taskkill to kill process tree (`#1725 <https://github.com/ros/ros_comm/issues/1725>`_)
* pass missing args (`#1733 <https://github.com/ros/ros_comm/issues/1733>`_)
* fix for roslaunch-check on Python 3
* roslaunch added --required option (`#1681 <https://github.com/ros/ros_comm/issues/1681>`_)
* more Python 3 compatibility (`#1783 <https://github.com/ros/ros_comm/issues/1783>`_)
* more Python 3 compatibility (`#1782 <https://github.com/ros/ros_comm/issues/1782>`_)
* switch to yaml.safe_load(_all) to prevent YAMLLoadWarning (`#1688 <https://github.com/ros/ros_comm/issues/1688>`_)
* fix $(dirname) for roslaunch-check (`#1624 <https://github.com/ros/ros_comm/issues/1624>`_)
* change how commands are executed (`#1628 <https://github.com/ros/ros_comm/issues/1628>`_)
* add option to hide summary from roslaunch output (`#1655 <https://github.com/ros/ros_comm/issues/1655>`_)
* make roslaunch-check respect arg remappings with command line argument (`#1653 <https://github.com/ros/ros_comm/issues/1653>`_)
* add POSIX flag for shlex.split() (`#1619 <https://github.com/ros/ros_comm/issues/1619>`_)
* respawn if process died while checking should_respawn() (`#1590 <https://github.com/ros/ros_comm/issues/1590>`_)
* add python prefix for python scripts when there is no .py extension (`#1589 <https://github.com/ros/ros_comm/issues/1589>`_)
* xmlloader: use continue instead of pass for args_only (`#1540 <https://github.com/ros/ros_comm/issues/1540>`_)
* fix various test problems (`#1601 <https://github.com/ros/ros_comm/issues/1601>`_)
* normalize strings to utf-8 before setting as environment variable (`#1593 <https://github.com/ros/ros_comm/issues/1593>`_)
* fix typos: awhile -> a while (`#1534 <https://github.com/ros/ros_comm/issues/1534>`_)
* add more useful helper text here to indicate that this might be a permission error (`#1568 <https://github.com/ros/ros_comm/issues/1568>`_)
* exclude unused args check if pass_all_args is set (`#1520 <https://github.com/ros/ros_comm/issues/1520>`_)
* add an option in XmlLoader to only load arg tags (`#1521 <https://github.com/ros/ros_comm/issues/1521>`_)
* update wiki.ros.org URLs (`#1536 <https://github.com/ros/ros_comm/issues/1536>`_)
* improve exception handling when resource is not found (`#1476 <https://github.com/ros/ros_comm/issues/1476>`_)
* fix issues when built or run on Windows (`#1466 <https://github.com/ros/ros_comm/issues/1466>`_)

1.16.0 (2022-02-23)
-------------------
* REMOVEME: Force to use python3 for now
* Handling eval tag inside yaml files (#16)
* 1.15.11
* 1.15.10
* Fix AttributeError isAlive (#2092)
  * fix: roscore -> AttributeError: 'ProcessMonitor' object has no attribute 'isAlive'
  * Fix AttributeError: 'Thread' object has no attribute 'isAlive'
  * Update remaining instances of isAlive to is_alive
  Keep mock method isAlive to keep backwards compatibility for users who happen to be using the mock object.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [Windows] Single quote string fix-up (#2051)
  * add split_with_single_quote_enclosed_fixup
  * address feedback.
  * revert the renaming.
* fix misspell. (#2066)
* Change error message (#2035)
  As far as I can tell, not being able to ping itself does not *cause* the issue (at least if ping refers to using the `ping` program to check if a host can be reached). Not sure if this wording is the optimal.
* Ignore underscores when parsing literal numeric values (#2022)
  * Ignore underscores when parsing literal numeric values
  Python 3, due to PEP515, allows numerical values to be (visually)
  grouped via underscores. However, here, we want to consider literals
  containing an underscore as a string.
  * add comment
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* update changelogs
* fix bad relative import (still python 2 style) (#1973)
* 1.15.7
* update changelogs
* fix NameError in launch error handling (#1965)
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* roslaunch: Allow passing _TIMEOUT_SIGINT and _TIMEOUT_SIGTERM as parameters (#1937)
  * Added --sigint-timeout and --sigterm-timeout parameters.
  * Fixed propagating sigint- and sigterm-timeout to remote nodes.
  * Added tests for sigint- and sigterm-timeout.
  * Decrease CPU load of timeout tests.
  * roslaunch: readded DEFAULT_TIMEOUT_SIGINT and DEFAULT_TIMEOUT_SIGTERM
  * roslaunch: --sigint_timeout, --sigterm-timeout take floats now
  * Fixing int to float in docs for sigint_timeout and sigterm_timeout
  * Reduced flakyness of test_roslaunch_parent and test_nodeprocess
  * Migrate test/signal_logger.py to Python 3.
  * roslaunch: Better exception handling, undoing whitespace changes.
  * roslaunch: Fix whitespace.
  Co-authored-by: Christopher Wecht <christopher.wecht@student.kit.edu>
* roslaunch-check: Search dir recursively (#1914)
  * roslaunch-check: Search dir recursively
  This will simply check all .launch files in the directory you pass, including
  subdirectories. I recently set up roslaunch-check on a CI server, and
  missed having a feature like this.
  However, the directory you pass must still be a ROS package
  or a subdirectory of a ROS package. If you have, say, a dir
  containing many packages, and you want to check every launch
  file in every package in that dir, then you'll need to invoke
  roslaunch-check once for each package.
  * roslaunch-check: Don't search hidden folders
* Sort printed nodes by namespace alphabetically (#1934)
* [roslaunch] remove pycrypto import (not used) (#1922)
  pycrypto used to be installed through rosbag. This was dropped in #1609
  making roslaunch on remote hosts fail.
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
* update test to pass with old and new yaml (#1893)
* 1.14.4
* update changelog
* Allow empty machine arg in node tag. (#1885)
  * Allow empty machine tag.
  Fixes #274.
  * Fix tests for machine arg in node tag.
  * Remove obsolete test.
  * Remove obsolete test
* Use double quotes for portable roslaunch-check command. (#1883)
* Bump CMake version to avoid CMP0048 warning (#1869)
* wrap env prefix with double quotes (#1810)
  * wrap env prefix with double quotes
  * remove unnecessary string literal prefix
* [roslaunch] Add ignore default args option to roslaunch-check (#1788)
  * Add ignore default args option to roslaunch-check
  * Add '-i' option for ignoring default arg check
  * Add ignore default args option for roslaunch_add_file_check
  * Refactor ignore default args logic
  * Add unit test for ignore-default-args option
  * Update depends.py with proper ignore_default_args logic
  * Refactor 'ignore-default-args' param to 'ignore-unset-args'
  * Update test_roslaunch_rlutil.py with test changes
* add platform check for --args code path (#1809)
* [windows] Workaround for Python 2 xmlrpc performance issues. (#1872)
  * workaround for Python 2 xmlrpc performance issues.
  * address the feedback.
* Allowed empty ssh password for remote launching (#1826)
* Escape drive as well as path separator (#1815)
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
* [Windows][melodic-devel] Skip `cat` related test cases on Windows build (#1724)
  * Skip `cat` related test cases on Windows build
  * revert unrelated changes.
  * style PEP 8
  * style PEP 8
  * fix wrong argument names.
* Taskkill process tree. (#1725)
* Missing args (#1733)
* Small fixes for roslaunch-check on Python 3.
* roslaunch added --required option (#1681)
* more Python 3 compatibility (#1783)
* more Python 3 compatibility (#1782)
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
* Fix $(dirname) for roslaunch-check. (#1624)
* change how commands are executed (#1628)
  * Use non-posix way to split command string.
  * Use shell feature for roslaunch command.
  * Resolved the merge again.
  * rework shell=True for Windows change (#51)
  * rework shell=True for Windows change
  * remove .exe handling, prepend python for python scripts instead
  * update Windows search logic
  * move comments around and remove unnecessary comments
  * minor refactor
  * file is a keyword in python2, rename as f
  * remove unnecessary \ at the end of line
  * use sys.executable to launch Python interpreter
  * follow inline comment style in PEP8
  * invert logic
* #1468 revisited: add option to hide summary from roslaunch output. (#1655)
  * add option to hide summary from roslaunch output.
  * roslaunch no_summary: default is False, avoid double negative
  * roslaunch/parent.py: moved new argument at the end, fixed documentation
  * move new keyword argument to the end
  * spelling
* #1455 reviseted: make roslaunch-check respect arg remappings with command line argument (#1653)
  * make roslaunch-check respect arg remappings with command line argument (:=)
  * added exmpale for commandline arg remappings,
  scripts/roslaunch-check resources/example.launch commandline_true_arg:=false should be fail
  * roslaunch: removed rospy run dependency
* add POSIX flag for shlex.split() (#1619)
  * Fix more shlex.split usage for Windows.
  * remove space between args
* respawn if process died while checking should_respawn() (#1590)
  * If process died when checking should_respawn, treat it as over-due and respawn (#32)
  * fix how the return value of should_respawn() is checked
* add python prefix for python scripts when there is no .py extension (#1589)
  * add python prefix for python script when there is no .py extension
  * consolidate and minimize change to existing code
  * update comment
  * revert unnecessary change from sys.platform to os.name
* roslaunch/xmlloader: use continue instead of pass for args_only (#1540)
* fix various test problems (#1601)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* normalize strings to utf-8 before setting as environment variable (#1593)
  * normalize the string to utf-8 before passing to environment block.
  * convert from unicode to string when setting env variable (#21)
* Fixed typos: awhile -> a while (#1534)
* Added more useful helper text here to indicate that this might be a permission error. (#1568)
* Fix #889 : Exclude unused args check if pass_all_args is set (#1520)
  * Exclude unused args check if pass_all_args is set
  * Add test for pass_all_args fix
* roslaunch: add an option in XmlLoader to only load arg tags (#1521)
  This fixes #1300. It's off/false by default, but it's used by `arg_dump`
  to stop roslaunch-autocomplete from failing if any arg has no default value.
* Update wiki.ros.org URLs (#1536)
* [roslaunch] Better exception handling when resource is not found. (#1476)
  * [roslaunch] Better exception handling when resource is not found.
  When `$(find pkg)` fail to return a resource in `arg` tag, `roslaunch` fails without printing useful output. With this PR it provides better explanation.
  Without this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  With this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: Make sure the following is found in ROS_PACKAGE_PATH: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  ```
  $ more /tmp/invalid_arg.launch
  <?xml version="1.0"?>
  <launch>
  <arg name="foo" default="$(find foo)/.config" />
  <arg name="baa" default="$(arg foo)/hoge.yaml" />
  </launch>
  ```
  * [roslaunch] Raise appropriate exception when a resource was not found.
  Applyin feedback https://github.com/ros/ros_comm/pull/1476#discussion_r209081340
  * shortened the exception message a bit
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: 5tan, Alessandro Tondo, Andreas Vinter-Hviid, Brutus The Tschiepel, Christophe, Christopher Wecht, Collin Scribner, Daniel Ingram, David V. Lu!!, Dirk Thomas, Gary Servin, Isaac I.Y. Saito, Jacob Perron, James Xu, Jochen Sprickerhof, Johnson Shih, Lucas Walter, Martijn Buijs, Martin Pecka, Mike Purvis, Morten Fyhn Amundsen, Sean Yen, Shane Loretz, Tom Moore, Victor Lamoine, Xaver Kroischke, betab0t, tomoya

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* 1.16.0
* Update changelogs
* REMOVEME: Force to use python3 for now
* Handling eval tag inside yaml files (#16)
* 1.15.11
* 1.15.10
* Fix AttributeError isAlive (#2092)
  * fix: roscore -> AttributeError: 'ProcessMonitor' object has no attribute 'isAlive'
  * Fix AttributeError: 'Thread' object has no attribute 'isAlive'
  * Update remaining instances of isAlive to is_alive
  Keep mock method isAlive to keep backwards compatibility for users who happen to be using the mock object.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [Windows] Single quote string fix-up (#2051)
  * add split_with_single_quote_enclosed_fixup
  * address feedback.
  * revert the renaming.
* fix misspell. (#2066)
* Change error message (#2035)
  As far as I can tell, not being able to ping itself does not *cause* the issue (at least if ping refers to using the `ping` program to check if a host can be reached). Not sure if this wording is the optimal.
* Ignore underscores when parsing literal numeric values (#2022)
  * Ignore underscores when parsing literal numeric values
  Python 3, due to PEP515, allows numerical values to be (visually)
  grouped via underscores. However, here, we want to consider literals
  containing an underscore as a string.
  * add comment
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* update changelogs
* fix bad relative import (still python 2 style) (#1973)
* 1.15.7
* update changelogs
* fix NameError in launch error handling (#1965)
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* roslaunch: Allow passing _TIMEOUT_SIGINT and _TIMEOUT_SIGTERM as parameters (#1937)
  * Added --sigint-timeout and --sigterm-timeout parameters.
  * Fixed propagating sigint- and sigterm-timeout to remote nodes.
  * Added tests for sigint- and sigterm-timeout.
  * Decrease CPU load of timeout tests.
  * roslaunch: readded DEFAULT_TIMEOUT_SIGINT and DEFAULT_TIMEOUT_SIGTERM
  * roslaunch: --sigint_timeout, --sigterm-timeout take floats now
  * Fixing int to float in docs for sigint_timeout and sigterm_timeout
  * Reduced flakyness of test_roslaunch_parent and test_nodeprocess
  * Migrate test/signal_logger.py to Python 3.
  * roslaunch: Better exception handling, undoing whitespace changes.
  * roslaunch: Fix whitespace.
  Co-authored-by: Christopher Wecht <christopher.wecht@student.kit.edu>
* roslaunch-check: Search dir recursively (#1914)
  * roslaunch-check: Search dir recursively
  This will simply check all .launch files in the directory you pass, including
  subdirectories. I recently set up roslaunch-check on a CI server, and
  missed having a feature like this.
  However, the directory you pass must still be a ROS package
  or a subdirectory of a ROS package. If you have, say, a dir
  containing many packages, and you want to check every launch
  file in every package in that dir, then you'll need to invoke
  roslaunch-check once for each package.
  * roslaunch-check: Don't search hidden folders
* Sort printed nodes by namespace alphabetically (#1934)
* [roslaunch] remove pycrypto import (not used) (#1922)
  pycrypto used to be installed through rosbag. This was dropped in #1609
  making roslaunch on remote hosts fail.
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
* update test to pass with old and new yaml (#1893)
* 1.14.4
* update changelog
* Allow empty machine arg in node tag. (#1885)
  * Allow empty machine tag.
  Fixes #274.
  * Fix tests for machine arg in node tag.
  * Remove obsolete test.
  * Remove obsolete test
* Use double quotes for portable roslaunch-check command. (#1883)
* Bump CMake version to avoid CMP0048 warning (#1869)
* wrap env prefix with double quotes (#1810)
  * wrap env prefix with double quotes
  * remove unnecessary string literal prefix
* [roslaunch] Add ignore default args option to roslaunch-check (#1788)
  * Add ignore default args option to roslaunch-check
  * Add '-i' option for ignoring default arg check
  * Add ignore default args option for roslaunch_add_file_check
  * Refactor ignore default args logic
  * Add unit test for ignore-default-args option
  * Update depends.py with proper ignore_default_args logic
  * Refactor 'ignore-default-args' param to 'ignore-unset-args'
  * Update test_roslaunch_rlutil.py with test changes
* add platform check for --args code path (#1809)
* [windows] Workaround for Python 2 xmlrpc performance issues. (#1872)
  * workaround for Python 2 xmlrpc performance issues.
  * address the feedback.
* Allowed empty ssh password for remote launching (#1826)
* Escape drive as well as path separator (#1815)
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
* [Windows][melodic-devel] Skip `cat` related test cases on Windows build (#1724)
  * Skip `cat` related test cases on Windows build
  * revert unrelated changes.
  * style PEP 8
  * style PEP 8
  * fix wrong argument names.
* Taskkill process tree. (#1725)
* Missing args (#1733)
* Small fixes for roslaunch-check on Python 3.
* roslaunch added --required option (#1681)
* more Python 3 compatibility (#1783)
* more Python 3 compatibility (#1782)
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
* Fix $(dirname) for roslaunch-check. (#1624)
* change how commands are executed (#1628)
  * Use non-posix way to split command string.
  * Use shell feature for roslaunch command.
  * Resolved the merge again.
  * rework shell=True for Windows change (#51)
  * rework shell=True for Windows change
  * remove .exe handling, prepend python for python scripts instead
  * update Windows search logic
  * move comments around and remove unnecessary comments
  * minor refactor
  * file is a keyword in python2, rename as f
  * remove unnecessary \ at the end of line
  * use sys.executable to launch Python interpreter
  * follow inline comment style in PEP8
  * invert logic
* #1468 revisited: add option to hide summary from roslaunch output. (#1655)
  * add option to hide summary from roslaunch output.
  * roslaunch no_summary: default is False, avoid double negative
  * roslaunch/parent.py: moved new argument at the end, fixed documentation
  * move new keyword argument to the end
  * spelling
* #1455 reviseted: make roslaunch-check respect arg remappings with command line argument (#1653)
  * make roslaunch-check respect arg remappings with command line argument (:=)
  * added exmpale for commandline arg remappings,
  scripts/roslaunch-check resources/example.launch commandline_true_arg:=false should be fail
  * roslaunch: removed rospy run dependency
* add POSIX flag for shlex.split() (#1619)
  * Fix more shlex.split usage for Windows.
  * remove space between args
* respawn if process died while checking should_respawn() (#1590)
  * If process died when checking should_respawn, treat it as over-due and respawn (#32)
  * fix how the return value of should_respawn() is checked
* add python prefix for python scripts when there is no .py extension (#1589)
  * add python prefix for python script when there is no .py extension
  * consolidate and minimize change to existing code
  * update comment
  * revert unnecessary change from sys.platform to os.name
* roslaunch/xmlloader: use continue instead of pass for args_only (#1540)
* fix various test problems (#1601)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* normalize strings to utf-8 before setting as environment variable (#1593)
  * normalize the string to utf-8 before passing to environment block.
  * convert from unicode to string when setting env variable (#21)
* Fixed typos: awhile -> a while (#1534)
* Added more useful helper text here to indicate that this might be a permission error. (#1568)
* Fix #889 : Exclude unused args check if pass_all_args is set (#1520)
  * Exclude unused args check if pass_all_args is set
  * Add test for pass_all_args fix
* roslaunch: add an option in XmlLoader to only load arg tags (#1521)
  This fixes #1300. It's off/false by default, but it's used by `arg_dump`
  to stop roslaunch-autocomplete from failing if any arg has no default value.
* Update wiki.ros.org URLs (#1536)
* [roslaunch] Better exception handling when resource is not found. (#1476)
  * [roslaunch] Better exception handling when resource is not found.
  When `$(find pkg)` fail to return a resource in `arg` tag, `roslaunch` fails without printing useful output. With this PR it provides better explanation.
  Without this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  With this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: Make sure the following is found in ROS_PACKAGE_PATH: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  ```
  $ more /tmp/invalid_arg.launch
  <?xml version="1.0"?>
  <launch>
  <arg name="foo" default="$(find foo)/.config" />
  <arg name="baa" default="$(arg foo)/hoge.yaml" />
  </launch>
  ```
  * [roslaunch] Raise appropriate exception when a resource was not found.
  Applyin feedback https://github.com/ros/ros_comm/pull/1476#discussion_r209081340
  * shortened the exception message a bit
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: 5tan, Alessandro Tondo, Andreas Vinter-Hviid, Brutus The Tschiepel, Christophe, Christopher Wecht, Collin Scribner, Daniel Ingram, David V. Lu!!, Dirk Thomas, Gary Servin, Isaac I.Y. Saito, Jacob Perron, James Xu, Jochen Sprickerhof, Johnson Shih, Lucas Walter, Martijn Buijs, Martin Pecka, Mike Purvis, Morten Fyhn Amundsen, Sean Yen, Shane Loretz, Tom Moore, Victor Lamoine, Xaver Kroischke, betab0t, tomoya

1.19.0 (2023-09-25)
-------------------
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* REMOVEME: Force to use python3 for now
* Handling eval tag inside yaml files (#16)
* 1.15.11
* 1.15.10
* Fix AttributeError isAlive (#2092)
  * fix: roscore -> AttributeError: 'ProcessMonitor' object has no attribute 'isAlive'
  * Fix AttributeError: 'Thread' object has no attribute 'isAlive'
  * Update remaining instances of isAlive to is_alive
  Keep mock method isAlive to keep backwards compatibility for users who happen to be using the mock object.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [Windows] Single quote string fix-up (#2051)
  * add split_with_single_quote_enclosed_fixup
  * address feedback.
  * revert the renaming.
* fix misspell. (#2066)
* Change error message (#2035)
  As far as I can tell, not being able to ping itself does not *cause* the issue (at least if ping refers to using the `ping` program to check if a host can be reached). Not sure if this wording is the optimal.
* Ignore underscores when parsing literal numeric values (#2022)
  * Ignore underscores when parsing literal numeric values
  Python 3, due to PEP515, allows numerical values to be (visually)
  grouped via underscores. However, here, we want to consider literals
  containing an underscore as a string.
  * add comment
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* update changelogs
* fix bad relative import (still python 2 style) (#1973)
* 1.15.7
* update changelogs
* fix NameError in launch error handling (#1965)
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* roslaunch: Allow passing _TIMEOUT_SIGINT and _TIMEOUT_SIGTERM as parameters (#1937)
  * Added --sigint-timeout and --sigterm-timeout parameters.
  * Fixed propagating sigint- and sigterm-timeout to remote nodes.
  * Added tests for sigint- and sigterm-timeout.
  * Decrease CPU load of timeout tests.
  * roslaunch: readded DEFAULT_TIMEOUT_SIGINT and DEFAULT_TIMEOUT_SIGTERM
  * roslaunch: --sigint_timeout, --sigterm-timeout take floats now
  * Fixing int to float in docs for sigint_timeout and sigterm_timeout
  * Reduced flakyness of test_roslaunch_parent and test_nodeprocess
  * Migrate test/signal_logger.py to Python 3.
  * roslaunch: Better exception handling, undoing whitespace changes.
  * roslaunch: Fix whitespace.
  Co-authored-by: Christopher Wecht <christopher.wecht@student.kit.edu>
* roslaunch-check: Search dir recursively (#1914)
  * roslaunch-check: Search dir recursively
  This will simply check all .launch files in the directory you pass, including
  subdirectories. I recently set up roslaunch-check on a CI server, and
  missed having a feature like this.
  However, the directory you pass must still be a ROS package
  or a subdirectory of a ROS package. If you have, say, a dir
  containing many packages, and you want to check every launch
  file in every package in that dir, then you'll need to invoke
  roslaunch-check once for each package.
  * roslaunch-check: Don't search hidden folders
* Sort printed nodes by namespace alphabetically (#1934)
* [roslaunch] remove pycrypto import (not used) (#1922)
  pycrypto used to be installed through rosbag. This was dropped in #1609
  making roslaunch on remote hosts fail.
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
* update test to pass with old and new yaml (#1893)
* 1.14.4
* update changelog
* Allow empty machine arg in node tag. (#1885)
  * Allow empty machine tag.
  Fixes #274.
  * Fix tests for machine arg in node tag.
  * Remove obsolete test.
  * Remove obsolete test
* Use double quotes for portable roslaunch-check command. (#1883)
* Bump CMake version to avoid CMP0048 warning (#1869)
* wrap env prefix with double quotes (#1810)
  * wrap env prefix with double quotes
  * remove unnecessary string literal prefix
* [roslaunch] Add ignore default args option to roslaunch-check (#1788)
  * Add ignore default args option to roslaunch-check
  * Add '-i' option for ignoring default arg check
  * Add ignore default args option for roslaunch_add_file_check
  * Refactor ignore default args logic
  * Add unit test for ignore-default-args option
  * Update depends.py with proper ignore_default_args logic
  * Refactor 'ignore-default-args' param to 'ignore-unset-args'
  * Update test_roslaunch_rlutil.py with test changes
* add platform check for --args code path (#1809)
* [windows] Workaround for Python 2 xmlrpc performance issues. (#1872)
  * workaround for Python 2 xmlrpc performance issues.
  * address the feedback.
* Allowed empty ssh password for remote launching (#1826)
* Escape drive as well as path separator (#1815)
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
* [Windows][melodic-devel] Skip `cat` related test cases on Windows build (#1724)
  * Skip `cat` related test cases on Windows build
  * revert unrelated changes.
  * style PEP 8
  * style PEP 8
  * fix wrong argument names.
* Taskkill process tree. (#1725)
* Missing args (#1733)
* Small fixes for roslaunch-check on Python 3.
* roslaunch added --required option (#1681)
* more Python 3 compatibility (#1783)
* more Python 3 compatibility (#1782)
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
* Fix $(dirname) for roslaunch-check. (#1624)
* change how commands are executed (#1628)
  * Use non-posix way to split command string.
  * Use shell feature for roslaunch command.
  * Resolved the merge again.
  * rework shell=True for Windows change (#51)
  * rework shell=True for Windows change
  * remove .exe handling, prepend python for python scripts instead
  * update Windows search logic
  * move comments around and remove unnecessary comments
  * minor refactor
  * file is a keyword in python2, rename as f
  * remove unnecessary \ at the end of line
  * use sys.executable to launch Python interpreter
  * follow inline comment style in PEP8
  * invert logic
* #1468 revisited: add option to hide summary from roslaunch output. (#1655)
  * add option to hide summary from roslaunch output.
  * roslaunch no_summary: default is False, avoid double negative
  * roslaunch/parent.py: moved new argument at the end, fixed documentation
  * move new keyword argument to the end
  * spelling
* #1455 reviseted: make roslaunch-check respect arg remappings with command line argument (#1653)
  * make roslaunch-check respect arg remappings with command line argument (:=)
  * added exmpale for commandline arg remappings,
  scripts/roslaunch-check resources/example.launch commandline_true_arg:=false should be fail
  * roslaunch: removed rospy run dependency
* add POSIX flag for shlex.split() (#1619)
  * Fix more shlex.split usage for Windows.
  * remove space between args
* respawn if process died while checking should_respawn() (#1590)
  * If process died when checking should_respawn, treat it as over-due and respawn (#32)
  * fix how the return value of should_respawn() is checked
* add python prefix for python scripts when there is no .py extension (#1589)
  * add python prefix for python script when there is no .py extension
  * consolidate and minimize change to existing code
  * update comment
  * revert unnecessary change from sys.platform to os.name
* roslaunch/xmlloader: use continue instead of pass for args_only (#1540)
* fix various test problems (#1601)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* normalize strings to utf-8 before setting as environment variable (#1593)
  * normalize the string to utf-8 before passing to environment block.
  * convert from unicode to string when setting env variable (#21)
* Fixed typos: awhile -> a while (#1534)
* Added more useful helper text here to indicate that this might be a permission error. (#1568)
* Fix #889 : Exclude unused args check if pass_all_args is set (#1520)
  * Exclude unused args check if pass_all_args is set
  * Add test for pass_all_args fix
* roslaunch: add an option in XmlLoader to only load arg tags (#1521)
  This fixes #1300. It's off/false by default, but it's used by `arg_dump`
  to stop roslaunch-autocomplete from failing if any arg has no default value.
* Update wiki.ros.org URLs (#1536)
* [roslaunch] Better exception handling when resource is not found. (#1476)
  * [roslaunch] Better exception handling when resource is not found.
  When `$(find pkg)` fail to return a resource in `arg` tag, `roslaunch` fails without printing useful output. With this PR it provides better explanation.
  Without this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  With this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: Make sure the following is found in ROS_PACKAGE_PATH: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  ```
  $ more /tmp/invalid_arg.launch
  <?xml version="1.0"?>
  <launch>
  <arg name="foo" default="$(find foo)/.config" />
  <arg name="baa" default="$(arg foo)/hoge.yaml" />
  </launch>
  ```
  * [roslaunch] Raise appropriate exception when a resource was not found.
  Applyin feedback https://github.com/ros/ros_comm/pull/1476#discussion_r209081340
  * shortened the exception message a bit
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: 5tan, Alessandro Tondo, Andreas Vinter-Hviid, Brutus The Tschiepel, Christophe, Christopher Wecht, Collin Scribner, Daniel Ingram, David V. Lu!!, Dirk Thomas, Gary Servin, Isaac I.Y. Saito, Jacob Perron, James Xu, Jochen Sprickerhof, Johnson Shih, Lucas Walter, Martijn Buijs, Martin Pecka, Mike Purvis, Morten Fyhn Amundsen, Sean Yen, Shane Loretz, Tom Moore, Victor Lamoine, Xaver Kroischke, betab0t, tomoya

1.21.1 (2024-10-08)
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
* 1.16.0
* Update changelogs
* REMOVEME: Force to use python3 for now
* Handling eval tag inside yaml files (#16)
* 1.15.11
* 1.15.10
* Fix AttributeError isAlive (#2092)
  * fix: roscore -> AttributeError: 'ProcessMonitor' object has no attribute 'isAlive'
  * Fix AttributeError: 'Thread' object has no attribute 'isAlive'
  * Update remaining instances of isAlive to is_alive
  Keep mock method isAlive to keep backwards compatibility for users who happen to be using the mock object.
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* [Windows] Single quote string fix-up (#2051)
  * add split_with_single_quote_enclosed_fixup
  * address feedback.
  * revert the renaming.
* fix misspell. (#2066)
* Change error message (#2035)
  As far as I can tell, not being able to ping itself does not *cause* the issue (at least if ping refers to using the `ping` program to check if a host can be reached). Not sure if this wording is the optimal.
* Ignore underscores when parsing literal numeric values (#2022)
  * Ignore underscores when parsing literal numeric values
  Python 3, due to PEP515, allows numerical values to be (visually)
  grouped via underscores. However, here, we want to consider literals
  containing an underscore as a string.
  * add comment
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* 1.15.8
* update changelogs
* fix bad relative import (still python 2 style) (#1973)
* 1.15.7
* update changelogs
* fix NameError in launch error handling (#1965)
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* roslaunch: Allow passing _TIMEOUT_SIGINT and _TIMEOUT_SIGTERM as parameters (#1937)
  * Added --sigint-timeout and --sigterm-timeout parameters.
  * Fixed propagating sigint- and sigterm-timeout to remote nodes.
  * Added tests for sigint- and sigterm-timeout.
  * Decrease CPU load of timeout tests.
  * roslaunch: readded DEFAULT_TIMEOUT_SIGINT and DEFAULT_TIMEOUT_SIGTERM
  * roslaunch: --sigint_timeout, --sigterm-timeout take floats now
  * Fixing int to float in docs for sigint_timeout and sigterm_timeout
  * Reduced flakyness of test_roslaunch_parent and test_nodeprocess
  * Migrate test/signal_logger.py to Python 3.
  * roslaunch: Better exception handling, undoing whitespace changes.
  * roslaunch: Fix whitespace.
  Co-authored-by: Christopher Wecht <christopher.wecht@student.kit.edu>
* roslaunch-check: Search dir recursively (#1914)
  * roslaunch-check: Search dir recursively
  This will simply check all .launch files in the directory you pass, including
  subdirectories. I recently set up roslaunch-check on a CI server, and
  missed having a feature like this.
  However, the directory you pass must still be a ROS package
  or a subdirectory of a ROS package. If you have, say, a dir
  containing many packages, and you want to check every launch
  file in every package in that dir, then you'll need to invoke
  roslaunch-check once for each package.
  * roslaunch-check: Don't search hidden folders
* Sort printed nodes by namespace alphabetically (#1934)
* [roslaunch] remove pycrypto import (not used) (#1922)
  pycrypto used to be installed through rosbag. This was dropped in #1609
  making roslaunch on remote hosts fail.
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
* update test to pass with old and new yaml (#1893)
* 1.14.4
* update changelog
* Allow empty machine arg in node tag. (#1885)
  * Allow empty machine tag.
  Fixes #274.
  * Fix tests for machine arg in node tag.
  * Remove obsolete test.
  * Remove obsolete test
* Use double quotes for portable roslaunch-check command. (#1883)
* Bump CMake version to avoid CMP0048 warning (#1869)
* wrap env prefix with double quotes (#1810)
  * wrap env prefix with double quotes
  * remove unnecessary string literal prefix
* [roslaunch] Add ignore default args option to roslaunch-check (#1788)
  * Add ignore default args option to roslaunch-check
  * Add '-i' option for ignoring default arg check
  * Add ignore default args option for roslaunch_add_file_check
  * Refactor ignore default args logic
  * Add unit test for ignore-default-args option
  * Update depends.py with proper ignore_default_args logic
  * Refactor 'ignore-default-args' param to 'ignore-unset-args'
  * Update test_roslaunch_rlutil.py with test changes
* add platform check for --args code path (#1809)
* [windows] Workaround for Python 2 xmlrpc performance issues. (#1872)
  * workaround for Python 2 xmlrpc performance issues.
  * address the feedback.
* Allowed empty ssh password for remote launching (#1826)
* Escape drive as well as path separator (#1815)
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
* [Windows][melodic-devel] Skip `cat` related test cases on Windows build (#1724)
  * Skip `cat` related test cases on Windows build
  * revert unrelated changes.
  * style PEP 8
  * style PEP 8
  * fix wrong argument names.
* Taskkill process tree. (#1725)
* Missing args (#1733)
* Small fixes for roslaunch-check on Python 3.
* roslaunch added --required option (#1681)
* more Python 3 compatibility (#1783)
* more Python 3 compatibility (#1782)
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
* Fix $(dirname) for roslaunch-check. (#1624)
* change how commands are executed (#1628)
  * Use non-posix way to split command string.
  * Use shell feature for roslaunch command.
  * Resolved the merge again.
  * rework shell=True for Windows change (#51)
  * rework shell=True for Windows change
  * remove .exe handling, prepend python for python scripts instead
  * update Windows search logic
  * move comments around and remove unnecessary comments
  * minor refactor
  * file is a keyword in python2, rename as f
  * remove unnecessary \ at the end of line
  * use sys.executable to launch Python interpreter
  * follow inline comment style in PEP8
  * invert logic
* #1468 revisited: add option to hide summary from roslaunch output. (#1655)
  * add option to hide summary from roslaunch output.
  * roslaunch no_summary: default is False, avoid double negative
  * roslaunch/parent.py: moved new argument at the end, fixed documentation
  * move new keyword argument to the end
  * spelling
* #1455 reviseted: make roslaunch-check respect arg remappings with command line argument (#1653)
  * make roslaunch-check respect arg remappings with command line argument (:=)
  * added exmpale for commandline arg remappings,
  scripts/roslaunch-check resources/example.launch commandline_true_arg:=false should be fail
  * roslaunch: removed rospy run dependency
* add POSIX flag for shlex.split() (#1619)
  * Fix more shlex.split usage for Windows.
  * remove space between args
* respawn if process died while checking should_respawn() (#1590)
  * If process died when checking should_respawn, treat it as over-due and respawn (#32)
  * fix how the return value of should_respawn() is checked
* add python prefix for python scripts when there is no .py extension (#1589)
  * add python prefix for python script when there is no .py extension
  * consolidate and minimize change to existing code
  * update comment
  * revert unnecessary change from sys.platform to os.name
* roslaunch/xmlloader: use continue instead of pass for args_only (#1540)
* fix various test problems (#1601)
  * move test files to separate package
  * move publishtest into separate package since it requires rostopic which rostest can't depend on
  * [rosbag_storage] add missing dependency on std_msgs
  * duplicate talker.py test node since rospy doesn't install the file
  * modify test to pass when rospy.get_name isn't available without a dependency declared on it
* normalize strings to utf-8 before setting as environment variable (#1593)
  * normalize the string to utf-8 before passing to environment block.
  * convert from unicode to string when setting env variable (#21)
* Fixed typos: awhile -> a while (#1534)
* Added more useful helper text here to indicate that this might be a permission error. (#1568)
* Fix #889 : Exclude unused args check if pass_all_args is set (#1520)
  * Exclude unused args check if pass_all_args is set
  * Add test for pass_all_args fix
* roslaunch: add an option in XmlLoader to only load arg tags (#1521)
  This fixes #1300. It's off/false by default, but it's used by `arg_dump`
  to stop roslaunch-autocomplete from failing if any arg has no default value.
* Update wiki.ros.org URLs (#1536)
* [roslaunch] Better exception handling when resource is not found. (#1476)
  * [roslaunch] Better exception handling when resource is not found.
  When `$(find pkg)` fail to return a resource in `arg` tag, `roslaunch` fails without printing useful output. With this PR it provides better explanation.
  Without this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  With this PR:
  ```
  $ roslaunch /tmp/invalid_arg.launch
  :
  Invalid <arg> tag: Make sure the following is found in ROS_PACKAGE_PATH: foo
  ROS path [0]=/opt/ros/kinetic/share/ros
  ROS path [1]=/home/n130s/ROS/indigo_trusty/cws_rosdt/src/ros/ros_comm/tools/roslaunch
  ROS path [2]=/opt/ros/kinetic/share
  Arg xml is <arg default="$(find foo)/.config" name="foo"/>
  The traceback for the exception was written to the log file
  ```
  ```
  $ more /tmp/invalid_arg.launch
  <?xml version="1.0"?>
  <launch>
  <arg name="foo" default="$(find foo)/.config" />
  <arg name="baa" default="$(arg foo)/hoge.yaml" />
  </launch>
  ```
  * [roslaunch] Raise appropriate exception when a resource was not found.
  Applyin feedback https://github.com/ros/ros_comm/pull/1476#discussion_r209081340
  * shortened the exception message a bit
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: 5tan, Alessandro Tondo, Andreas Vinter-Hviid, Brutus The Tschiepel, Christophe, Christopher Wecht, Collin Scribner, Daniel Ingram, David V. Lu!!, Dirk Thomas, Gary Servin, Isaac I.Y. Saito, Jacob Perron, James Xu, Jochen Sprickerhof, Johnson Shih, Lucas Walter, Martijn Buijs, Martin Pecka, Mike Purvis, Morten Fyhn Amundsen, Sean Yen, Shane Loretz, Tom Moore, Victor Lamoine, Xaver Kroischke, betab0t, tomoya

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* fix "pass_all_args" for roslaunch-check, add nosetest (`#1368 <https://github.com/ros/ros_comm/issues/1368>`_)
* add --log option to roslaunch (`#1330 <https://github.com/ros/ros_comm/issues/1330>`_)
* add substitution when loading yaml files (`#1354 <https://github.com/ros/ros_comm/issues/1354>`_)

1.13.6 (2018-02-05)
-------------------
* add process listeners to XML RPC server (`#1319 <https://github.com/ros/ros_comm/issues/1319>`_)
* pass through command-line args to the xmlloader when using the API (`#1115 <https://github.com/ros/ros_comm/issues/1115>`_)
* make master process explicitly 'required' for parent launch (`#1228 <https://github.com/ros/ros_comm/issues/1228>`_)
* remove unreachable exceptions (`#1260 <https://github.com/ros/ros_comm/issues/1260>`_)
* replace Thread.setDaemon() using new API (`#1276 <https://github.com/ros/ros_comm/issues/1276>`_)
* use roslaunch.core.printerrlog for printing error message (`#1193 <https://github.com/ros/ros_comm/issues/1193>`_, `#1317 <https://github.com/ros/ros_comm/issues/1317>`_)

1.13.5 (2017-11-09)
-------------------

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* add --set-master-logger-level option for 'rosmaster' to output LOG_API (`#1180 <https://github.com/ros/ros_comm/issues/1180>`_)
* use defined error codes rather than hardcoded integers (`#1174 <https://github.com/ros/ros_comm/issues/1174>`_, `#1181 <https://github.com/ros/ros_comm/issues/1181>`_)
* fix parameter leaking into sibling scopes (`#1158 <https://github.com/ros/ros_comm/issues/1158>`_)
* avoid full stack trace for ResourceNotFound (`#1147 <https://github.com/ros/ros_comm/issues/1147>`_)
* remove mention of rosmake from error message (`#1140 <https://github.com/ros/ros_comm/issues/1140>`_)

1.13.2 (2017-08-15)
-------------------

1.13.1 (2017-07-27)
-------------------
* add $(dirname) to get directory of current launch file (`#1103 <https://github.com/ros/ros_comm/pull/1103>`_)
* clean the namespace to get rid of double or trailing forward slashes (`#1100 <https://github.com/ros/ros_comm/issues/1100>`_)
* only launch core nodes if master was launched by roslaunch (`#1098 <https://github.com/ros/ros_comm/pull/1098>`_)
* ensure pid file is removed on exit (`#1057 <https://github.com/ros/ros_comm/pull/1057>`_, `#1084 <https://github.com/ros/ros_comm/pull/1084>`_)
* add yaml type for param tag (`#1045 <https://github.com/ros/ros_comm/issues/1045>`_)
* ensure cwd exists (`#1031 <https://github.com/ros/ros_comm/pull/1031>`_)
* respect if/unless for roslaunch-check (`#998 <https://github.com/ros/ros_comm/pull/998>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* improve error message for invalid tags (`#989 <https://github.com/ros/ros_comm/pull/989>`_)
* fix caching logic to improve performance (`#931 <https://github.com/ros/ros_comm/pull/931>`_)

1.12.6 (2016-10-26)
-------------------
* add USE_TEST_DEPENDENCIES option to roslaunch_add_file_check() (`#910 <https://github.com/ros/ros_comm/pull/910>`_)

1.12.5 (2016-09-30)
-------------------

1.12.4 (2016-09-19)
-------------------

1.12.3 (2016-09-17)
-------------------
* better naming for roslaunch check test results (`#897 <https://github.com/ros/ros_comm/pull/897>`_)
* support use_test_depends option for roslaunch-check (`#887 <https://github.com/ros/ros_comm/pull/887>`_)
* allow empty include (`#882 <https://github.com/ros/ros_comm/pull/882>`_)
* fix param command for Python 3 (`#840 <https://github.com/ros/ros_comm/pull/840>`_)

1.12.2 (2016-06-03)
-------------------
* support registering the same test multiple times with different arguments (`#814 <https://github.com/ros/ros_comm/pull/814>`_)
* fix passing multiple args to roslaunch_add_file_check (`#814 <https://github.com/ros/ros_comm/pull/814>`_)

1.12.1 (2016-04-18)
-------------------
* add support for Python expressions (`#784 <https://github.com/ros/ros_comm/pull/784>`_, `#793 <https://github.com/ros/ros_comm/pull/793>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------
* improve roslaunch-check to not fail if recursive dependencies lack dependencies (`#730 <https://github.com/ros/ros_comm/pull/730>`_)
* add "pass_all_args" attribute to roslaunch "include" tag (`#710 <https://github.com/ros/ros_comm/pull/710>`_)
* fix a typo in unknown host error message (`#735 <https://github.com/ros/ros_comm/pull/735>`_)
* wait for param server to be available before trying to get param (`#711 <https://github.com/ros/ros_comm/pull/711>`_)

1.11.16 (2015-11-09)
--------------------
* add `-w` and `-t` options (`#687 <https://github.com/ros/ros_comm/pull/687>`_)
* fix missing minimum version for rospkg dependency (`#693 <https://github.com/ros/ros_comm/issues/693>`_)

1.11.15 (2015-10-13)
--------------------
* improve performance by reusing the rospack instance across nodes with the same default environment (`#682 <https://github.com/ros/ros_comm/pull/682>`_)

1.11.14 (2015-09-19)
--------------------
* add more information when test times out

1.11.13 (2015-04-28)
--------------------

1.11.12 (2015-04-27)
--------------------

1.11.11 (2015-04-16)
--------------------

1.11.10 (2014-12-22)
--------------------
* fix exception at roscore startup if python has IPv6 disabled (`#515 <https://github.com/ros/ros_comm/issues/515>`_)
* fix error handling (`#516 <https://github.com/ros/ros_comm/pull/516>`_)
* fix compatibility with paramiko 1.10.0 (`#498 <https://github.com/ros/ros_comm/pull/498>`_)

1.11.9 (2014-08-18)
-------------------
* fix usage of logger before it is initialized (`#490 <https://github.com/ros/ros_comm/issues/490>`_) (regression from 1.11.6)

1.11.8 (2014-08-04)
-------------------
* remove implicit rostest dependency and use rosunit instead (`#475 <https://github.com/ros/ros_comm/issues/475>`_)
* accept stdin input alongside files (`#472 <https://github.com/ros/ros_comm/issues/472>`_)

1.11.7 (2014-07-18)
-------------------
* fix the ROS_MASTER_URI environment variable logic on Windows (`#2 <https://github.com/windows/ros_comm/issues/2>`_)

1.11.6 (2014-07-10)
-------------------
* fix printing of non-ascii roslaunch parameters (`#454 <https://github.com/ros/ros_comm/issues/454>`_)
* add respawn_delay attribute to node tag in roslaunch (`#446 <https://github.com/ros/ros_comm/issues/446>`_)
* write traceback for exceptions in roslaunch to log file

1.11.5 (2014-06-24)
-------------------

1.11.4 (2014-06-16)
-------------------
* fix handling of if/unless attributes on args (`#437 <https://github.com/ros/ros_comm/issues/437>`_)
* improve parameter printing in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* Python 3 compatibility (`#426 <https://github.com/ros/ros_comm/issues/426>`_, `#427 <https://github.com/ros/ros_comm/issues/427>`_, `#429 <https://github.com/ros/ros_comm/issues/429>`_)

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix roslaunch anonymous function to generate the same output for the same input (`#297 <https://github.com/ros/ros_comm/issues/297>`_)
* add doc attribute to roslaunch arg tags (`#379 <https://github.com/ros/ros_comm/issues/379>`_)
* print parameter values in roslaunch (`#89 <https://github.com/ros/ros_comm/issues/89>`_)
* add architecture_independent flag in package.xml (`#391 <https://github.com/ros/ros_comm/issues/391>`_)

1.11.0 (2014-03-04)
-------------------
* use catkin_install_python() to install Python scripts (`#361 <https://github.com/ros/ros_comm/issues/361>`_)

1.10.0 (2014-02-11)
-------------------
* add optional DEPENDENCIES argument to roslaunch_add_file_check()
* add explicit run dependency (`#347 <https://github.com/ros/ros_comm/issues/347>`_)

1.9.54 (2014-01-27)
-------------------
* add missing run/test dependencies on rosbuild to get ROS_ROOT environment variable

1.9.53 (2014-01-14)
-------------------

1.9.52 (2014-01-08)
-------------------

1.9.51 (2014-01-07)
-------------------
* fix roslaunch-check for unreleased wet dependencies (`#332 <https://github.com/ros/ros_comm/issues/332>`_)

1.9.50 (2013-10-04)
-------------------
* add option to disable terminal title setting
* fix roslaunch-check to handle more complex launch files

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------
* update roslaunch to support ROS_NAMESPACE (`#58 <https://github.com/ros/ros_comm/issues/58>`_)
* make roslaunch relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)
* change roslaunch resolve order (`#256 <https://github.com/ros/ros_comm/issues/256>`_)
* fix roslaunch check script in install space (`#257 <https://github.com/ros/ros_comm/issues/257>`_)

1.9.47 (2013-07-03)
-------------------
* improve roslaunch completion to include launch file arguments (`#230 <https://github.com/ros/ros_comm/issues/230>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.9.46 (2013-06-18)
-------------------
* add CMake function roslaunch_add_file_check() (`#241 <https://github.com/ros/ros_comm/issues/241>`_)

1.9.45 (2013-06-06)
-------------------
* modified roslaunch $(find PKG) to consider path behind it for resolve strategy (`#233 <https://github.com/ros/ros_comm/pull/233>`_)
* add boolean attribute 'subst_value' to rosparam tag in launch files (`#218 <https://github.com/ros/ros_comm/issues/218>`_)
* add command line parameter to print out launch args
* fix missing import in arg_dump.py

1.9.44 (2013-03-21)
-------------------
* fix 'roslaunch --files' with non-unique anononymous ids (`#186 <https://github.com/ros/ros_comm/issues/186>`_)
* fix ROS_MASTER_URI for Windows

1.9.43 (2013-03-13)
-------------------
* implement process killer for Windows

1.9.42 (2013-03-08)
-------------------
* add option --skip-log-check (`#133 <https://github.com/ros/ros_comm/issues/133>`_)
* update API doc to list raised exceptions in config.py
* fix invocation of Python scripts under Windows (`#54 <https://github.com/ros/ros_comm/issues/54>`_)

1.9.41 (2013-01-24)
-------------------
* improve performance of $(find ...)

1.9.40 (2013-01-13)
-------------------
* fix 'roslaunch --pid=' when pointing to ROS_HOME but folder does not exist (`#43 <https://github.com/ros/ros_comm/issues/43>`_)
* fix 'roslaunch --pid=' to use shell expansion for the pid value (`#44 <https://github.com/ros/ros_comm/issues/44>`_)

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
