^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosbag_storage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
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
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* Contributors: Dirk Thomas, Jacob Perron, Sean Yen, Shane Loretz

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
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
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Sean Yen, Shane Loretz

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
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Sean Yen, Shane Loretz

1.24.0 (2026-09-21)
-------------------
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
* 1.16.0
* 1.15.15
* Move @jacobperron from maintainer to author (#2302)
* 1.15.14
* 1.15.13
* 1.15.12
* Contributors: Jacob Perron, Michael Carroll, Robert Haschke, Shane Loretz

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
* 1.15.7
* 1.15.6
* 1.15.5
* 1.15.4
* 1.15.3
* 1.15.2
* 1.15.1
* 1.15.0
* 1.14.4
* Bump CMake version to avoid CMP0048 warning (#1869)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Sean Yen, Shane Loretz

1.9.0 (2022-02-23)
-------------------
