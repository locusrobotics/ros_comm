#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, Locus Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Locus Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Integration tests that exercise the C++ rosmaster_cpp via XML-RPC."""

import os
import socket
import sys
import unittest

# Ensure the test directory is on sys.path so master_integration can be found
# regardless of the working directory (e.g., when run by nose in CI).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from master_integration import (
    CoreAPIMixin,
    MasterProcess,
    ParamServerMixin,
    RegistrationsMixin,
    find_cpp_master,
    find_free_port,
)

socket.setdefaulttimeout(5.0)

_master = None
_skip_reason = None

try:
    _cpp_binary = find_cpp_master()
except RuntimeError as e:
    _cpp_binary = None
    _skip_reason = str(e)


def setUpModule():
    global _master
    if _cpp_binary is None:
        raise unittest.SkipTest(_skip_reason)
    _master = MasterProcess(_cpp_binary, find_free_port())


def tearDownModule():
    global _master
    if _master:
        _master.stop()
        _master = None


class _Base(unittest.TestCase):
    def setUp(self):
        if _master is None:
            self.skipTest('C++ master not available')
        self.proxy = _master.proxy
        self.master_port = _master.port
        self.master_uri = _master.uri


class TestCppCoreAPI(_Base, CoreAPIMixin):
    pass


class TestCppParamServer(_Base, ParamServerMixin):
    pass


class TestCppRegistrations(_Base, RegistrationsMixin):
    pass


if __name__ == '__main__':
    unittest.main()
