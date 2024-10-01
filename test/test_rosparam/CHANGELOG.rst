^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosparam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2022-02-23)
-------------------
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Fix paramtest for empty values of a parameter (#2054)
  * Fix paramtest for empty values of a parameter (ros#2053)
  * Fix paramtest for empty values of a parameter (ros#2053)
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
* Contributors: Dirk Thomas, Jacob Perron, Levko Ivanchuk, Martijn Buijs, Sean Yen, Shane Loretz

1.18.0 (2023-02-22)
-------------------
* 1.17.0
* Fix changelog
* 1.16.0
* Update changelogs
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Fix paramtest for empty values of a parameter (#2054)
  * Fix paramtest for empty values of a parameter (ros#2053)
  * Fix paramtest for empty values of a parameter (ros#2053)
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
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Levko Ivanchuk, Martijn Buijs, Sean Yen, Shane Loretz

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
* Fix paramtest for empty values of a parameter (#2054)
  * Fix paramtest for empty values of a parameter (ros#2053)
  * Fix paramtest for empty values of a parameter (ros#2053)
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
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Levko Ivanchuk, Martijn Buijs, Sean Yen, Shane Loretz

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
* 1.15.11
* 1.15.10
* 1.15.9 package.xmls
* Fix paramtest for empty values of a parameter (#2054)
  * Fix paramtest for empty values of a parameter (ros#2053)
  * Fix paramtest for empty values of a parameter (ros#2053)
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
* Contributors: Dirk Thomas, Gary Servin, Jacob Perron, Levko Ivanchuk, Martijn Buijs, Sean Yen, Shane Loretz

1.9.0 (2022-02-23)
-------------------
