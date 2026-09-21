^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosservice
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Update maintainers (#2075)
  Previous: @dirk-thomas
  New: @jacobperron, @mjcarroll, @sloretz
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
* [Windows][melodic-devel] Make test code to be more portable (#1726)
  * Make test code to be more portable.
  * unrelated change
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
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Dirk Thomas, Jacob Perron, Martijn Buijs, Sean Yen, Shane Loretz

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
* [Windows][melodic-devel] Make test code to be more portable (#1726)
  * Make test code to be more portable.
  * unrelated change
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
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Martijn Buijs, Sean Yen, Shane Loretz

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
* [Windows][melodic-devel] Make test code to be more portable (#1726)
  * Make test code to be more portable.
  * unrelated change
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
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Martijn Buijs, Sean Yen, Shane Loretz

1.24.0 (2026-09-21)
-------------------
* Fixes for Python 3.12
  With fixes by Jochen Sprickerhof.
  Taken from
  https://salsa.debian.org/science-team/ros-ros-comm/-/blob/b74ca5c2c868a084ab36e46d68f7775518ac4c58/debian/patches/0016-Fixes-for-Python-3.12.patch
  (cherry picked from commit 42cd22e509907d1e89765f8d1a27cbb201321d28)
* 1.17.0
* 1.16.0
* 1.15.15
* Move @jacobperron from maintainer to author (#2302)
* 1.15.14
* 1.15.13
* 1.15.12
* Contributors: Jacob Perron, Matthias Klose, Michael Carroll, Shane Loretz

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
* [Windows][melodic-devel] Make test code to be more portable (#1726)
  * Make test code to be more portable.
  * unrelated change
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
* duplicate test nodes which aren't available to other packages, add missing dependencies (#1611)
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Martijn Buijs, Sean Yen, Shane Loretz

1.9.0 (2022-02-23)
-------------------
