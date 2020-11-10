^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_roslaunch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.0 (2020-11-10)
-------------------

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
* [Windows][melodic-devel] Skip `cat` related test cases on Windows build (`#1724 <https://github.com/locusrobotics/ros_comm/issues/1724>`_)
  * Skip `cat` related test cases on Windows build
  * revert unrelated changes.
  * style PEP 8
  * style PEP 8
  * fix wrong argument names.
* duplicate test nodes which aren't available to other packages, add missing dependencies (`#1611 <https://github.com/locusrobotics/ros_comm/issues/1611>`_)
* Contributors: Dirk Thomas, Sean Yen
