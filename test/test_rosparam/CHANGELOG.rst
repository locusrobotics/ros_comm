^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosparam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* use condition attributes to specify Python 2 and 3 dependencies (`#1792 <https://github.com/locusrobotics/ros_comm/issues/1792>`_)
  * use condition attributes to specify Python 2 and 3 dependencies
  * use python3-pil
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
* Contributors: Dirk Thomas, Martijn Buijs
