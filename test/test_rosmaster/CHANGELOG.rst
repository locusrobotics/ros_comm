^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosmaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.0 (2020-10-02)
-------------------
* more Python 3 compatibility (`#1795 <https://github.com/locusrobotics/ros_comm/issues/1795>`_)
  * avoid using nose.tools without dependency being declared
  * seek(0)
  * subprocess decode
  * import urlparse
  * fix hash arg encode
  * print function
  * replace tabs used for indenting Python code with spaces
* more Python 3 compatibility (`#1782 <https://github.com/locusrobotics/ros_comm/issues/1782>`_)
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
* Fixed typos: awhile -> a while (`#1534 <https://github.com/locusrobotics/ros_comm/issues/1534>`_)
* Update wiki.ros.org URLs (`#1536 <https://github.com/locusrobotics/ros_comm/issues/1536>`_)
* Contributors: Daniel Ingram, Dirk Thomas, Martijn Buijs, Victor Lamoine
