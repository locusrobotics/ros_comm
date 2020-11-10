^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rostopic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.15.0 (2020-10-02)
-------------------
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
* Contributors: Martijn Buijs
