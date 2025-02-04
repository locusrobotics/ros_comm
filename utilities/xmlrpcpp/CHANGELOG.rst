^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xmlrpcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.11 (2021-04-06)
--------------------

1.15.10 (2021-03-18)
--------------------
* Portable fix to recent Windows build breaks (`#2110 <https://github.com/ros/ros_comm/issues/2110>`_)
* Contributors: Sean Yen

1.15.9 (2020-10-16)
-------------------
* Update maintainers (`#2075 <https://github.com/ros/ros_comm/issues/2075>`_)
* Fix to address CVE-2020-16124 (`#2065 <https://github.com/ros/ros_comm/issues/2065>`_)
* Fix spelling (`#2066 <https://github.com/ros/ros_comm/issues/2066>`_)
* Fix XmlRpcValue::_doubleFormat being unused (`#2003 <https://github.com/ros/ros_comm/issues/2003>`_)
* Contributors: Shane Loretz, Sid Faber, tomoya

1.15.8 (2020-07-23)
-------------------
* add const versions of XmlRpcValue converting operators (`#1978 <https://github.com/ros/ros_comm/issues/1978>`_)

1.15.7 (2020-05-28)
-------------------

1.15.6 (2020-05-21)
-------------------

1.15.5 (2020-05-15)
-------------------
* check if enough FDs are free, instead counting the total free FDs (`#1929 <https://github.com/ros/ros_comm/issues/1929>`_)

1.15.4 (2020-03-19)
-------------------
* restrict boost dependencies to components used (`#1871 <https://github.com/ros/ros_comm/issues/1871>`_)

1.15.3 (2020-02-28)
-------------------

1.15.2 (2020-02-25)
-------------------

1.15.1 (2020-02-24)
-------------------

1.15.0 (2020-02-21)
-------------------

1.14.4 (2020-02-20)
-------------------
* bump CMake minimum version to avoid CMP0048 warning (`#1869 <https://github.com/ros/ros_comm/issues/1869>`_)
* [Windows] workaround WSAPoll doesn't report failed connections (`#1816 <https://github.com/ros/ros_comm/issues/1816>`_)
* fix base64 decode error on ARM platforms (`#1853 <https://github.com/ros/ros_comm/issues/1853>`_)
* use c++11 std::snprintf (`#1820 <https://github.com/ros/ros_comm/issues/1820>`_)
* fix dead loop if accept connection error in XmlRpcServer (`#1791 <https://github.com/ros/ros_comm/issues/1791>`_)
* fix test build errors (`#1723 <https://github.com/ros/ros_comm/issues/1723>`_)
* fix base64 encode error (`#1769 <https://github.com/ros/ros_comm/issues/1769>`_)
* XmlRpcValue added bool assignment operator (`#1709 <https://github.com/ros/ros_comm/issues/1709>`_)
* add const indexer for xmlrpc (`#1759 <https://github.com/ros/ros_comm/issues/1759>`_)
* xmlrpcpp: fixed invalid zero index (`#1631 <https://github.com/ros/ros_comm/issues/1631>`_)
* avoid calling memcpy on NULL pointer with size 0 (`#1546 <https://github.com/ros/ros_comm/issues/1546>`_)
* revert "Revert "move the winsock2.h into cpp."" (`#1588 <https://github.com/ros/ros_comm/issues/1588>`_)
* visibility macros update (`#1591 <https://github.com/ros/ros_comm/issues/1591>`_)
* remove explicit -std=c++11, default to 14
* fix test code build issues on Windows (`#1479 <https://github.com/ros/ros_comm/issues/1479>`_)
* fix issues when built or run on Windows (`#1466 <https://github.com/ros/ros_comm/issues/1466>`_)

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* Portable fix to recent Windows build breaks. (#2110)
  * Portable fix.
  * include <climits>
  * Adding missing headers.
  * Adding #include <vector>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* Trap for overly large input to XmlRPCPP (#2065)
  * Trap for overly large input to XmlRPCPP which could cause problems with int <-> size_t conversions.
  - In XmlRpcClient, XmlRpcServerConnection and XmlRpcSocket, recognize when incoming or outgoing data is too large, generate an error and discard the data when practical.
  - Use the safe strtol() rather than atoi() to decode an incoming content-length header, and generate an error if the length is invalid or too large.
  - In XmlRpcUtil, prevent attempts to parse overly large XML input.
  - Add tests where they can reasonably be inserted into existing test routines.
  Although this fix could be cleaner the update is written to make the update ABI compatible.
  This fix addresses CVE-2020-16124 / Integer overflow in ros_comm.
  * Trap for memory allocation error in tests
  * Revert earlier change
  * Update tests
  Replace call to GTEST_SKIP with output to stderr. Remove the
  redResponseOversize test since out-of-memory errors during
  testing cannot easily be handled within the existing test objects.
  * Improve test error handling
  Use GTEST_SKIP if available, otherwise print to stderr. Remove test
  that's being killed because it takes too long to handle the oversize
  test values
* fix misspell. (#2066)
* XmlRpcValue::_doubleFormat should be used during write. (#2003)
  * XmlRpcValue::_doubleFormat should be used during write.
  * allocate buffer dynamically for XmlRpcValue::_doubleFormat if necessary.
  * add test for XmlRpcValue::_doubleFormat.
  * check return code from std::snprintf, save/restore DoubleFormat for test.
  * add one time warning message for DoubleFormat.
  * use XmlRpcUtil::error instead of ROS_ERROR.
  * use static_cast and minor fixes.
  * delete unrelated change and fix invalid format case.
  * get rid of redundant condition from if statement.
* 1.15.8
* update changelogs
* Added const versions of XmlRpcValue converting operators. (#1978)
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Check if enough FDs are free, instead counting the total free FDs. (#1929)
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
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* [Windows][melodic-devel] workaround WSAPoll doesn't report failed connections. (#1816)
  * workaround wsapoll not report failed connections.
  * add link for reference.
* Fix base64 decode error on ARM platforms (#1853)
  * Revert "fix base64 encode error (#1769)"
  This reverts commit ec8b0ef34a7d8063066af27aee33ac559b3043d1.
  * libb64: integer overflows
  Copied from https://sourceforge.net/p/libb64/bugs/2/:
  > The attached patch fixes integers overflows in the decoder.
  > The first hunk is needed for systems with signed chars (e.g. i386).
  > The other hunks fix the decoder on unsigned-char systems (on which it's currently completely broken).
  Co-authored-by: Jonathan Wakely <github@kayari.org>
* use c++11 std::snprintf (#1820)
* Fix dead loop if accept connection error in XmlRpcServer (#1791)
* Fixed test build errors. (#1723)
* fix base64 encode error (#1769)
* XmlRpcValue added bool assignment operator (#1709)
* Added const indexer for xmlrpc (#1759)
* xmlrpcpp: fixed invalid zero index as suggested in #1547 (#1631)
* Avoid calling memcpy on NULL pointer with size 0. (#1546)
* Revert "Revert "move the winsock2.h into cpp."" (#1588)
  This reverts commit e3e2bfbd75615b52231956285eeddc7fd2cf22c1.
* visibility macros update (#1591)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix test code build issues on Windows (#1479)
  * Fix test code build issues on Windows
  * Change not to touch XmlRpcClient header, expose the enum from test code
  * Add missing header file back
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Alex Moriarty, Christopher Wecht, Dirk Thomas, Hanno Böck, Jacob Perron, James Xu, Jason Wang, Johannes Meyer, Johnson Shih, Martin Pecka, Mikael Arguedas, Sean Yen, Shane Loretz, Sid Faber, randoms, tomoya

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Portable fix to recent Windows build breaks. (#2110)
  * Portable fix.
  * include <climits>
  * Adding missing headers.
  * Adding #include <vector>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* Trap for overly large input to XmlRPCPP (#2065)
  * Trap for overly large input to XmlRPCPP which could cause problems with int <-> size_t conversions.
  - In XmlRpcClient, XmlRpcServerConnection and XmlRpcSocket, recognize when incoming or outgoing data is too large, generate an error and discard the data when practical.
  - Use the safe strtol() rather than atoi() to decode an incoming content-length header, and generate an error if the length is invalid or too large.
  - In XmlRpcUtil, prevent attempts to parse overly large XML input.
  - Add tests where they can reasonably be inserted into existing test routines.
  Although this fix could be cleaner the update is written to make the update ABI compatible.
  This fix addresses CVE-2020-16124 / Integer overflow in ros_comm.
  * Trap for memory allocation error in tests
  * Revert earlier change
  * Update tests
  Replace call to GTEST_SKIP with output to stderr. Remove the
  redResponseOversize test since out-of-memory errors during
  testing cannot easily be handled within the existing test objects.
  * Improve test error handling
  Use GTEST_SKIP if available, otherwise print to stderr. Remove test
  that's being killed because it takes too long to handle the oversize
  test values
* fix misspell. (#2066)
* XmlRpcValue::_doubleFormat should be used during write. (#2003)
  * XmlRpcValue::_doubleFormat should be used during write.
  * allocate buffer dynamically for XmlRpcValue::_doubleFormat if necessary.
  * add test for XmlRpcValue::_doubleFormat.
  * check return code from std::snprintf, save/restore DoubleFormat for test.
  * add one time warning message for DoubleFormat.
  * use XmlRpcUtil::error instead of ROS_ERROR.
  * use static_cast and minor fixes.
  * delete unrelated change and fix invalid format case.
  * get rid of redundant condition from if statement.
* 1.15.8
* update changelogs
* Added const versions of XmlRpcValue converting operators. (#1978)
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Check if enough FDs are free, instead counting the total free FDs. (#1929)
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
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* [Windows][melodic-devel] workaround WSAPoll doesn't report failed connections. (#1816)
  * workaround wsapoll not report failed connections.
  * add link for reference.
* Fix base64 decode error on ARM platforms (#1853)
  * Revert "fix base64 encode error (#1769)"
  This reverts commit ec8b0ef34a7d8063066af27aee33ac559b3043d1.
  * libb64: integer overflows
  Copied from https://sourceforge.net/p/libb64/bugs/2/:
  > The attached patch fixes integers overflows in the decoder.
  > The first hunk is needed for systems with signed chars (e.g. i386).
  > The other hunks fix the decoder on unsigned-char systems (on which it's currently completely broken).
  Co-authored-by: Jonathan Wakely <github@kayari.org>
* use c++11 std::snprintf (#1820)
* Fix dead loop if accept connection error in XmlRpcServer (#1791)
* Fixed test build errors. (#1723)
* fix base64 encode error (#1769)
* XmlRpcValue added bool assignment operator (#1709)
* Added const indexer for xmlrpc (#1759)
* xmlrpcpp: fixed invalid zero index as suggested in #1547 (#1631)
* Avoid calling memcpy on NULL pointer with size 0. (#1546)
* Revert "Revert "move the winsock2.h into cpp."" (#1588)
  This reverts commit e3e2bfbd75615b52231956285eeddc7fd2cf22c1.
* visibility macros update (#1591)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix test code build issues on Windows (#1479)
  * Fix test code build issues on Windows
  * Change not to touch XmlRpcClient header, expose the enum from test code
  * Add missing header file back
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Alex Moriarty, Christopher Wecht, Dirk Thomas, Gary Servin, Hanno Böck, Jacob Perron, James Xu, Jason Wang, Johannes Meyer, Johnson Shih, Martin Pecka, Mikael Arguedas, Sean Yen, Shane Loretz, Sid Faber, randoms, tomoya

1.19.0 (2023-09-25)
-------------------
* 1.18.0
* Update changelogs
* 1.17.0
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Portable fix to recent Windows build breaks. (#2110)
  * Portable fix.
  * include <climits>
  * Adding missing headers.
  * Adding #include <vector>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* Trap for overly large input to XmlRPCPP (#2065)
  * Trap for overly large input to XmlRPCPP which could cause problems with int <-> size_t conversions.
  - In XmlRpcClient, XmlRpcServerConnection and XmlRpcSocket, recognize when incoming or outgoing data is too large, generate an error and discard the data when practical.
  - Use the safe strtol() rather than atoi() to decode an incoming content-length header, and generate an error if the length is invalid or too large.
  - In XmlRpcUtil, prevent attempts to parse overly large XML input.
  - Add tests where they can reasonably be inserted into existing test routines.
  Although this fix could be cleaner the update is written to make the update ABI compatible.
  This fix addresses CVE-2020-16124 / Integer overflow in ros_comm.
  * Trap for memory allocation error in tests
  * Revert earlier change
  * Update tests
  Replace call to GTEST_SKIP with output to stderr. Remove the
  redResponseOversize test since out-of-memory errors during
  testing cannot easily be handled within the existing test objects.
  * Improve test error handling
  Use GTEST_SKIP if available, otherwise print to stderr. Remove test
  that's being killed because it takes too long to handle the oversize
  test values
* fix misspell. (#2066)
* XmlRpcValue::_doubleFormat should be used during write. (#2003)
  * XmlRpcValue::_doubleFormat should be used during write.
  * allocate buffer dynamically for XmlRpcValue::_doubleFormat if necessary.
  * add test for XmlRpcValue::_doubleFormat.
  * check return code from std::snprintf, save/restore DoubleFormat for test.
  * add one time warning message for DoubleFormat.
  * use XmlRpcUtil::error instead of ROS_ERROR.
  * use static_cast and minor fixes.
  * delete unrelated change and fix invalid format case.
  * get rid of redundant condition from if statement.
* 1.15.8
* update changelogs
* Added const versions of XmlRpcValue converting operators. (#1978)
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Check if enough FDs are free, instead counting the total free FDs. (#1929)
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
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* [Windows][melodic-devel] workaround WSAPoll doesn't report failed connections. (#1816)
  * workaround wsapoll not report failed connections.
  * add link for reference.
* Fix base64 decode error on ARM platforms (#1853)
  * Revert "fix base64 encode error (#1769)"
  This reverts commit ec8b0ef34a7d8063066af27aee33ac559b3043d1.
  * libb64: integer overflows
  Copied from https://sourceforge.net/p/libb64/bugs/2/:
  > The attached patch fixes integers overflows in the decoder.
  > The first hunk is needed for systems with signed chars (e.g. i386).
  > The other hunks fix the decoder on unsigned-char systems (on which it's currently completely broken).
  Co-authored-by: Jonathan Wakely <github@kayari.org>
* use c++11 std::snprintf (#1820)
* Fix dead loop if accept connection error in XmlRpcServer (#1791)
* Fixed test build errors. (#1723)
* fix base64 encode error (#1769)
* XmlRpcValue added bool assignment operator (#1709)
* Added const indexer for xmlrpc (#1759)
* xmlrpcpp: fixed invalid zero index as suggested in #1547 (#1631)
* Avoid calling memcpy on NULL pointer with size 0. (#1546)
* Revert "Revert "move the winsock2.h into cpp."" (#1588)
  This reverts commit e3e2bfbd75615b52231956285eeddc7fd2cf22c1.
* visibility macros update (#1591)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix test code build issues on Windows (#1479)
  * Fix test code build issues on Windows
  * Change not to touch XmlRpcClient header, expose the enum from test code
  * Add missing header file back
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Alex Moriarty, Christopher Wecht, Dirk Thomas, Gary Servin, Hanno Böck, Jacob Perron, James Xu, Jason Wang, Johannes Meyer, Johnson Shih, Martin Pecka, Mikael Arguedas, Sean Yen, Shane Loretz, Sid Faber, randoms, tomoya

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
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* Portable fix to recent Windows build breaks. (#2110)
  * Portable fix.
  * include <climits>
  * Adding missing headers.
  * Adding #include <vector>
* 1.15.9 package.xmls
* 1.15.9
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
* Trap for overly large input to XmlRPCPP (#2065)
  * Trap for overly large input to XmlRPCPP which could cause problems with int <-> size_t conversions.
  - In XmlRpcClient, XmlRpcServerConnection and XmlRpcSocket, recognize when incoming or outgoing data is too large, generate an error and discard the data when practical.
  - Use the safe strtol() rather than atoi() to decode an incoming content-length header, and generate an error if the length is invalid or too large.
  - In XmlRpcUtil, prevent attempts to parse overly large XML input.
  - Add tests where they can reasonably be inserted into existing test routines.
  Although this fix could be cleaner the update is written to make the update ABI compatible.
  This fix addresses CVE-2020-16124 / Integer overflow in ros_comm.
  * Trap for memory allocation error in tests
  * Revert earlier change
  * Update tests
  Replace call to GTEST_SKIP with output to stderr. Remove the
  redResponseOversize test since out-of-memory errors during
  testing cannot easily be handled within the existing test objects.
  * Improve test error handling
  Use GTEST_SKIP if available, otherwise print to stderr. Remove test
  that's being killed because it takes too long to handle the oversize
  test values
* fix misspell. (#2066)
* XmlRpcValue::_doubleFormat should be used during write. (#2003)
  * XmlRpcValue::_doubleFormat should be used during write.
  * allocate buffer dynamically for XmlRpcValue::_doubleFormat if necessary.
  * add test for XmlRpcValue::_doubleFormat.
  * check return code from std::snprintf, save/restore DoubleFormat for test.
  * add one time warning message for DoubleFormat.
  * use XmlRpcUtil::error instead of ROS_ERROR.
  * use static_cast and minor fixes.
  * delete unrelated change and fix invalid format case.
  * get rid of redundant condition from if statement.
* 1.15.8
* update changelogs
* Added const versions of XmlRpcValue converting operators. (#1978)
* 1.15.7
* update changelogs
* 1.15.6
* update changelogs
* 1.15.5
* update changelogs
* Check if enough FDs are free, instead counting the total free FDs. (#1929)
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
* 1.15.2
* update changelogs
* 1.15.1
* update changelogs
* 1.15.0
* update changelogs
* 1.14.4
* update changelog
* Bump CMake version to avoid CMP0048 warning (#1869)
* [Windows][melodic-devel] workaround WSAPoll doesn't report failed connections. (#1816)
  * workaround wsapoll not report failed connections.
  * add link for reference.
* Fix base64 decode error on ARM platforms (#1853)
  * Revert "fix base64 encode error (#1769)"
  This reverts commit ec8b0ef34a7d8063066af27aee33ac559b3043d1.
  * libb64: integer overflows
  Copied from https://sourceforge.net/p/libb64/bugs/2/:
  > The attached patch fixes integers overflows in the decoder.
  > The first hunk is needed for systems with signed chars (e.g. i386).
  > The other hunks fix the decoder on unsigned-char systems (on which it's currently completely broken).
  Co-authored-by: Jonathan Wakely <github@kayari.org>
* use c++11 std::snprintf (#1820)
* Fix dead loop if accept connection error in XmlRpcServer (#1791)
* Fixed test build errors. (#1723)
* fix base64 encode error (#1769)
* XmlRpcValue added bool assignment operator (#1709)
* Added const indexer for xmlrpc (#1759)
* xmlrpcpp: fixed invalid zero index as suggested in #1547 (#1631)
* Avoid calling memcpy on NULL pointer with size 0. (#1546)
* Revert "Revert "move the winsock2.h into cpp."" (#1588)
  This reverts commit e3e2bfbd75615b52231956285eeddc7fd2cf22c1.
* visibility macros update (#1591)
* [C++14] remove explicit -std=c++11, default to 14 (#1525)
  From REP-0003
  > Melodic
  > As of ROS Melodic, we are using the C++14 (ISO/IEC 14882:2014) standard.
  Related to discussion on ros-planning/moveit#1146
* Fix test code build issues on Windows (#1479)
  * Fix test code build issues on Windows
  * Change not to touch XmlRpcClient header, expose the enum from test code
  * Add missing header file back
* Fix issues when built or run on Windows (#1466)
  * Fix roslz4 build issue on Windows
  * Fix xmlrpcpp build issue on Windows, fix polling fails when run on Windows
  * Fix roscpp build issue on Windows
  * Fix rosbag_storage build issue on Windows
  * fix issues in python scripts to run roscore on Windows
  * revert unrelated whitespace changes
  * Revert changes in roslogging.py
  * declare const for source_cnt
* Contributors: Alex Moriarty, Christopher Wecht, Dirk Thomas, Gary Servin, Hanno Böck, Jacob Perron, James Xu, Jason Wang, Johannes Meyer, Johnson Shih, Martin Pecka, Mikael Arguedas, Sean Yen, Shane Loretz, Sid Faber, randoms, tomoya

1.14.3 (2018-08-06)
-------------------

1.14.2 (2018-06-06)
-------------------

1.14.1 (2018-05-21)
-------------------

1.14.0 (2018-05-21)
-------------------
* fixes for OSX (`#1402 <https://github.com/ros/ros_comm/issues/1402>`_)
* take XmlRpcValue by *const* ref. in operator<< (`#1350 <https://github.com/ros/ros_comm/issues/1350>`_)
* fix various compiler warnings on bionic (`#1325 <https://github.com/ros/ros_comm/issues/1325>`_)

1.13.6 (2018-02-05)
-------------------
* fix xmlrpc timeout using monotonic clock (`#1249 <https://github.com/ros/ros_comm/issues/1249>`_)
* add tests and bug fixes for XmlRpcServer (`#1243 <https://github.com/ros/ros_comm/issues/1243>`_)
* add test and fix uninitialized data in XmlRpcClient (`#1244 <https://github.com/ros/ros_comm/issues/1244>`_)
* make xmlrpcpp specific include directory work in devel space (`#1261 <https://github.com/ros/ros_comm/issues/1261>`_)
* add base64 tests (`#1242 <https://github.com/ros/ros_comm/issues/1242>`_)
* add unit tests for XmlRpcDispatch (`#1232 <https://github.com/ros/ros_comm/issues/1232>`_)
* add unit tests and bug fixes for XmlRpcClient (`#1221 <https://github.com/ros/ros_comm/issues/1221>`_)

1.13.5 (2017-11-09)
-------------------
* add unit tests and bug fixes for XmlRpcSocket (`#1218 <https://github.com/ros/ros_comm/issues/1218>`_)
* add tests for XmlRpcValue and XML conversion (`#1216 <https://github.com/ros/ros_comm/issues/1216>`_)

1.13.4 (2017-11-02)
-------------------

1.13.3 (2017-10-25)
-------------------
* fix problem when configuring tests without gtest being available (`#1197 <https://github.com/ros/ros_comm/issues/1197>`_)

1.13.2 (2017-08-15)
-------------------
* use poll() in favor of select() in the XmlRPCDispatcher (`#833 <https://github.com/ros/ros_comm/issues/833>`_)
* fix fall through warnings with g++ 7 (`#1139 <https://github.com/ros/ros_comm/issues/1139>`_)

1.13.1 (2017-07-27)
-------------------
* switch to libb64 for base64 encoding/decoding (`#1046 <https://github.com/ros/ros_comm/issues/1046>`_)

1.13.0 (2017-02-22)
-------------------

1.12.7 (2017-02-17)
-------------------
* move headers to include/xmlrpcpp (`#962 <https://github.com/ros/ros_comm/issues/962>`_)

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
* use directory specific compiler flags (`#785 <https://github.com/ros/ros_comm/pull/785>`_)

1.12.0 (2016-03-18)
-------------------

1.11.18 (2016-03-17)
--------------------

1.11.17 (2016-03-11)
--------------------

1.11.16 (2015-11-09)
--------------------

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
* improve Android support (`#537 <https://github.com/ros/ros_comm/pull/537>`_)
* fix various defects reported by coverity

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

1.11.3 (2014-05-21)
-------------------

1.11.2 (2014-05-08)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix day comparison for rpc value of type timestamp (`#395 <https://github.com/ros/ros_comm/issues/395>`_)

1.11.0 (2014-03-04)
-------------------
* output error message when hostname lookup fails (`#364 <https://github.com/ros/ros_comm/issues/364>`_)

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
* fix compilation and warnings with clang (`#291 <https://github.com/ros/ros_comm/issues/291>`_)

1.9.50 (2013-10-04)
-------------------

1.9.49 (2013-09-16)
-------------------

1.9.48 (2013-08-21)
-------------------

1.9.47 (2013-07-03)
-------------------

1.9.46 (2013-06-18)
-------------------

1.9.45 (2013-06-06)
-------------------

1.9.44 (2013-03-21)
-------------------
* fix install destination for dll's under Windows

1.9.43 (2013-03-13)
-------------------

1.9.42 (2013-03-08)
-------------------
* refine license description to LGPL-2.1

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-29)
-------------------
* first public release for Groovy
