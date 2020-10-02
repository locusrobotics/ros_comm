^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_rosservice
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* duplicate test nodes which aren't available to other packages, add missing dependencies (`#1611 <https://github.com/locusrobotics/ros_comm/issues/1611>`_)
* Contributors: Dirk Thomas, Martijn Buijs
