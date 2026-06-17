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

"""
Shared utilities and test mixin classes for rosmaster integration tests.

This module provides:
  - MasterProcess: manages a rosmaster subprocess lifecycle
  - Test mixin classes (CoreAPIMixin, ParamServerMixin, RegistrationsMixin)
    containing test methods that work against any master implementation

The mixin classes expect the following attributes on `self`:
  self.proxy       - xmlrpc.client.ServerProxy connected to the master
  self.master_port - port the master is listening on
  self.master_uri  - full URI of the master
"""

import os
import shutil
import signal
import socket
import subprocess
import time
import xmlrpc.client


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def find_free_port():
    """Bind to port 0 and let the OS assign a free port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('localhost', 0))
        return s.getsockname()[1]


def wait_for_master(proxy, timeout=10.0):
    """Poll the master until it responds to getPid."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            code, _, _ = proxy.getPid('/probe')
            if code == 1:
                return True
        except Exception:
            pass
        time.sleep(0.1)
    raise RuntimeError("Master did not become ready within %.1fs" % timeout)


def find_python_master():
    """Return the path to the Python rosmaster script."""
    path = shutil.which('rosmaster')
    if path:
        return path
    raise RuntimeError("Cannot find 'rosmaster' on PATH")


def find_cpp_master():
    """Return the path to the C++ rosmaster_cpp binary."""
    try:
        import roslib.packages
        matches = roslib.packages.find_node('rosmaster', 'rosmaster_cpp')
        if matches:
            return matches[0]
    except Exception:
        pass
    raise RuntimeError("Cannot find 'rosmaster_cpp' binary in rosmaster package")


class MasterProcess:
    """Context-managed rosmaster subprocess."""

    def __init__(self, binary_path, port):
        self.port = port
        self.uri = 'http://localhost:%d/' % port
        self.proc = subprocess.Popen(
            [binary_path, '--core', '-p', str(port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.proxy = xmlrpc.client.ServerProxy(self.uri)
        wait_for_master(self.proxy)

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()


# ---------------------------------------------------------------------------
# Test Mixin: Core API
# ---------------------------------------------------------------------------

class CoreAPIMixin:

    def test_get_pid(self):
        code, _, pid = self.proxy.getPid('/test')
        self.assertEqual(code, 1)
        self.assertIsInstance(pid, int)
        self.assertGreater(pid, 0)

    def test_get_uri(self):
        code, _, uri = self.proxy.getUri('/test')
        self.assertEqual(code, 1)
        self.assertTrue(uri.startswith('http://'))
        self.assertIn(str(self.master_port), uri)


# ---------------------------------------------------------------------------
# Test Mixin: Parameter Server
# ---------------------------------------------------------------------------

class ParamServerMixin:

    def _pname(self, suffix=''):
        """Return a unique parameter name based on the current test method."""
        return '/' + self._testMethodName + suffix

    # -- basic types --------------------------------------------------------

    def test_set_get_int(self):
        key = self._pname()
        code, _, _ = self.proxy.setParam('/test', key, 42)
        self.assertEqual(code, 1)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, 42)

    def test_set_get_string(self):
        key = self._pname()
        self.proxy.setParam('/test', key, 'hello world')
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, 'hello world')

    def test_set_get_float(self):
        key = self._pname()
        self.proxy.setParam('/test', key, 3.14)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertAlmostEqual(val, 3.14, places=5)

    def test_set_get_bool(self):
        key = self._pname()
        self.proxy.setParam('/test', key, True)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertTrue(val)
        self.proxy.setParam('/test', key, False)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertFalse(val)

    def test_set_get_list(self):
        key = self._pname()
        data = [1, 'two', 3.0, True]
        self.proxy.setParam('/test', key, data)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, data)

    def test_set_get_dict(self):
        key = self._pname()
        data = {'a': 1, 'b': 'hello', 'c': [1, 2]}
        self.proxy.setParam('/test', key, data)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, data)

    def test_set_get_nested_dict(self):
        key = self._pname()
        data = {'level1': {'level2': {'value': 42}}}
        self.proxy.setParam('/test', key, data)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, data)

    def test_set_get_empty_string(self):
        key = self._pname()
        self.proxy.setParam('/test', key, '')
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, '')

    # -- has / delete / names -----------------------------------------------

    def test_has_param(self):
        key = self._pname()
        self.proxy.setParam('/test', key, 1)
        code, _, has = self.proxy.hasParam('/test', key)
        self.assertEqual(code, 1)
        self.assertTrue(has)

    def test_has_param_missing(self):
        code, _, has = self.proxy.hasParam('/test', '/nonexistent_xyz_has')
        self.assertEqual(code, 1)
        self.assertFalse(has)

    def test_delete_param(self):
        key = self._pname()
        self.proxy.setParam('/test', key, 'to_delete')
        code, _, _ = self.proxy.deleteParam('/test', key)
        self.assertEqual(code, 1)
        code, _, has = self.proxy.hasParam('/test', key)
        self.assertFalse(has)

    def test_delete_param_subtree(self):
        """Deleting a namespace removes all children."""
        ns = self._pname()
        self.proxy.setParam('/test', ns + '/a', 1)
        self.proxy.setParam('/test', ns + '/b', 2)
        code, _, _ = self.proxy.deleteParam('/test', ns)
        self.assertEqual(code, 1)
        code, _, has = self.proxy.hasParam('/test', ns + '/a')
        self.assertFalse(has)

    def test_delete_param_missing(self):
        code, _, _ = self.proxy.deleteParam('/test', '/nonexistent_xyz_del')
        self.assertEqual(code, -1)

    def test_get_param_names(self):
        key = self._pname()
        self.proxy.setParam('/test', key, 'val')
        code, _, names = self.proxy.getParamNames('/test')
        self.assertEqual(code, 1)
        self.assertIn(key, names)

    def test_get_param_missing(self):
        code, _, _ = self.proxy.getParam('/test', '/nonexistent_xyz_get')
        self.assertEqual(code, -1)

    # -- namespaces and trees -----------------------------------------------

    def test_param_namespace_tree(self):
        """Set params in a namespace and retrieve as a dict tree."""
        ns = self._pname()
        self.proxy.setParam('/test', ns + '/x', 1)
        self.proxy.setParam('/test', ns + '/y', 2)
        self.proxy.setParam('/test', ns + '/z', 3)
        code, _, tree = self.proxy.getParam('/test', ns)
        self.assertEqual(code, 1)
        self.assertIsInstance(tree, dict)
        self.assertEqual(tree['x'], 1)
        self.assertEqual(tree['y'], 2)
        self.assertEqual(tree['z'], 3)

    def test_search_param(self):
        """searchParam walks up the namespace tree."""
        key = self._pname()
        self.proxy.setParam('/test', key, 'found')
        # Search from a deep namespace — should find it at root
        code, _, found_key = self.proxy.searchParam('/deep/ns/node', key.lstrip('/'))
        self.assertEqual(code, 1)
        self.assertEqual(found_key, key)

    def test_search_param_not_found(self):
        code, _, _ = self.proxy.searchParam('/test', 'nonexistent_search_xyz')
        self.assertEqual(code, -1)

    # -- overwrite ----------------------------------------------------------

    def test_param_overwrite_type(self):
        """Overwriting a param with a different type should succeed."""
        key = self._pname()
        self.proxy.setParam('/test', key, 42)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(val, 42)

        self.proxy.setParam('/test', key, 'now_a_string')
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, 'now_a_string')

    def test_param_overwrite_dict_with_scalar(self):
        """Overwriting a dict namespace with a scalar should work."""
        key = self._pname()
        self.proxy.setParam('/test', key, {'a': 1, 'b': 2})
        self.proxy.setParam('/test', key, 99)
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, 99)

    # -- subscribe / unsubscribe --------------------------------------------

    def test_subscribe_param(self):
        """subscribeParam should return the current value."""
        key = self._pname()
        self.proxy.setParam('/test', key, 'current')
        code, _, val = self.proxy.subscribeParam(
            '/caller', 'http://localhost:12345', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, 'current')

    def test_subscribe_param_nonexistent(self):
        """subscribeParam for a key that doesn't exist should return an empty dict, not crash."""
        key = self._pname()
        code, _, val = self.proxy.subscribeParam(
            '/caller', 'http://localhost:12345', key)
        self.assertEqual(code, 1)
        self.assertIsInstance(val, dict)
        # Must not contain internal sentinel keys
        for k in val:
            self.assertNotIn(k, ('__placeholder__', '__init__', '__empty__'))

    def test_get_param_dict_no_sentinels(self):
        """getParam on a dict must not leak internal sentinel keys."""
        key = self._pname()
        self.proxy.setParam('/test', key, {'a': 1, 'b': 2})
        code, _, val = self.proxy.getParam('/test', key)
        self.assertEqual(code, 1)
        self.assertEqual(val, {'a': 1, 'b': 2})
        for k in val:
            self.assertNotIn(k, ('__placeholder__', '__init__', '__empty__'))

    def test_get_param_after_delete_sibling_no_sentinels(self):
        """After deleting one child, the parent dict must not contain sentinel keys."""
        base = self._pname()
        self.proxy.setParam('/test', base + '/child1', 'v1')
        self.proxy.setParam('/test', base + '/child2', 'v2')
        self.proxy.deleteParam('/test', base + '/child1')
        code, _, val = self.proxy.getParam('/test', base)
        self.assertEqual(code, 1)
        self.assertIn('child2', val)
        for k in val:
            self.assertNotIn(k, ('__placeholder__', '__init__', '__empty__'))

    def test_unsubscribe_param(self):
        key = self._pname()
        self.proxy.subscribeParam('/caller', 'http://localhost:12345', key)
        code, _, count = self.proxy.unsubscribeParam(
            '/caller', 'http://localhost:12345', key)
        self.assertEqual(code, 1)


# ---------------------------------------------------------------------------
# Test Mixin: Registrations
# ---------------------------------------------------------------------------

class RegistrationsMixin:

    def _node(self, name='node1'):
        return '/' + self._testMethodName + '_' + name

    def _topic(self, name='topic1'):
        return '/' + self._testMethodName + '_' + name

    def _service(self, name='srv1'):
        return '/' + self._testMethodName + '_' + name

    def _api(self, port=12345):
        return 'http://localhost:%d/' % port

    # -- publishers ---------------------------------------------------------

    def test_register_publisher(self):
        code, _, subs = self.proxy.registerPublisher(
            self._node(), self._topic(), 'std_msgs/String', self._api())
        self.assertEqual(code, 1)
        self.assertIsInstance(subs, list)

    def test_unregister_publisher(self):
        node, topic, api = self._node(), self._topic(), self._api()
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', api)
        code, _, count = self.proxy.unregisterPublisher(node, topic, api)
        self.assertEqual(code, 1)

    # -- subscribers --------------------------------------------------------

    def test_register_subscriber(self):
        code, _, pubs = self.proxy.registerSubscriber(
            self._node(), self._topic(), 'std_msgs/String', self._api())
        self.assertEqual(code, 1)
        self.assertIsInstance(pubs, list)

    def test_unregister_subscriber(self):
        node, topic, api = self._node(), self._topic(), self._api()
        self.proxy.registerSubscriber(node, topic, 'std_msgs/String', api)
        code, _, count = self.proxy.unregisterSubscriber(node, topic, api)
        self.assertEqual(code, 1)

    # -- services -----------------------------------------------------------

    def test_register_service(self):
        code, _, _ = self.proxy.registerService(
            self._node(), self._service(), 'rosrpc://localhost:9999', self._api())
        self.assertEqual(code, 1)

    def test_unregister_service(self):
        node, svc = self._node(), self._service()
        self.proxy.registerService(node, svc, 'rosrpc://localhost:9999', self._api())
        code, _, count = self.proxy.unregisterService(
            node, svc, 'rosrpc://localhost:9999')
        self.assertEqual(code, 1)

    # -- lookups ------------------------------------------------------------

    def test_lookup_node(self):
        node, topic = self._node(), self._topic()
        api = self._api(23456)
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', api)
        code, _, uri = self.proxy.lookupNode('/test', node)
        self.assertEqual(code, 1)
        self.assertEqual(uri, api)

    def test_lookup_node_missing(self):
        code, _, _ = self.proxy.lookupNode('/test', '/nonexistent_node_xyz')
        self.assertEqual(code, -1)

    def test_lookup_service(self):
        node, svc = self._node(), self._service()
        svc_api = 'rosrpc://localhost:8888'
        self.proxy.registerService(node, svc, svc_api, self._api())
        code, _, uri = self.proxy.lookupService('/test', svc)
        self.assertEqual(code, 1)
        self.assertEqual(uri, svc_api)

    def test_lookup_service_missing(self):
        code, _, _ = self.proxy.lookupService('/test', '/nonexistent_svc_xyz')
        self.assertEqual(code, -1)

    # -- graph queries ------------------------------------------------------

    def test_get_published_topics(self):
        node, topic = self._node(), self._topic()
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', self._api())
        code, _, topics = self.proxy.getPublishedTopics('/test', '')
        self.assertEqual(code, 1)
        topic_names = [t[0] for t in topics]
        self.assertIn(topic, topic_names)

    def test_get_topic_types(self):
        node, topic = self._node(), self._topic()
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', self._api())
        code, _, types = self.proxy.getTopicTypes('/test')
        self.assertEqual(code, 1)
        type_dict = {t[0]: t[1] for t in types}
        self.assertIn(topic, type_dict)
        self.assertEqual(type_dict[topic], 'std_msgs/String')

    def test_get_system_state(self):
        node, topic, svc = self._node(), self._topic(), self._service()
        api = self._api(34567)
        sub_node = self._node('sub')
        sub_api = self._api(34568)

        self.proxy.registerPublisher(node, topic, 'std_msgs/String', api)
        self.proxy.registerSubscriber(sub_node, topic, 'std_msgs/String', sub_api)
        self.proxy.registerService(node, svc, 'rosrpc://localhost:7777', api)

        code, _, state = self.proxy.getSystemState('/test')
        self.assertEqual(code, 1)
        self.assertEqual(len(state), 3)
        pubs, subs, services = state

        pub_dict = {e[0]: e[1] for e in pubs}
        self.assertIn(topic, pub_dict)
        self.assertIn(node, pub_dict[topic])

        sub_dict = {e[0]: e[1] for e in subs}
        self.assertIn(topic, sub_dict)
        self.assertIn(sub_node, sub_dict[topic])

        svc_dict = {e[0]: e[1] for e in services}
        self.assertIn(svc, svc_dict)

    # -- cross-registration -------------------------------------------------

    def test_register_publisher_returns_subscribers(self):
        """registerPublisher should return the list of existing subscriber APIs."""
        topic = self._topic()
        sub_api = self._api(45678)
        self.proxy.registerSubscriber(
            self._node('sub'), topic, 'std_msgs/String', sub_api)
        code, _, subs = self.proxy.registerPublisher(
            self._node('pub'), topic, 'std_msgs/String', self._api(45679))
        self.assertEqual(code, 1)
        self.assertIn(sub_api, subs)

    def test_register_subscriber_returns_publishers(self):
        """registerSubscriber should return the list of existing publisher APIs."""
        topic = self._topic()
        pub_api = self._api(56789)
        self.proxy.registerPublisher(
            self._node('pub'), topic, 'std_msgs/String', pub_api)
        code, _, pubs = self.proxy.registerSubscriber(
            self._node('sub'), topic, 'std_msgs/String', self._api(56790))
        self.assertEqual(code, 1)
        self.assertIn(pub_api, pubs)

    def test_node_reregistration(self):
        """Re-registering a node with a new API URI should update lookupNode."""
        node, topic = self._node(), self._topic()
        old_api = self._api(11111)
        new_api = self._api(22222)
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', old_api)
        self.proxy.registerPublisher(node, topic, 'std_msgs/String', new_api)
        code, _, uri = self.proxy.lookupNode('/test', node)
        self.assertEqual(code, 1)
        self.assertEqual(uri, new_api)

    def test_get_published_topics_with_subgraph(self):
        """getPublishedTopics with a subgraph filter."""
        node = self._node()
        t1 = '/' + self._testMethodName + '/ns/topic_a'
        t2 = '/' + self._testMethodName + '/other/topic_b'
        self.proxy.registerPublisher(node, t1, 'std_msgs/String', self._api(61111))
        self.proxy.registerPublisher(node, t2, 'std_msgs/String', self._api(61111))

        code, _, topics = self.proxy.getPublishedTopics(
            '/test', '/' + self._testMethodName + '/ns')
        self.assertEqual(code, 1)
        topic_names = [t[0] for t in topics]
        self.assertIn(t1, topic_names)
        self.assertNotIn(t2, topic_names)
