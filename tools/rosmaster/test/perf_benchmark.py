#!/usr/bin/env python3
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
Performance benchmarks for rosmaster: Python vs C++.

Simulates a large fleet of robots, each with many topics and services,
then measures the latency of key XML-RPC API calls that underpin
commands like `rostopic list` and `rosservice list`.

Usage:
    python3 perf_benchmark.py [--robots N] [--topics-per-robot N] [--services-per-robot N]

Runs both masters back-to-back and prints a comparison table.
"""

import argparse
import signal
import socket
import subprocess
import statistics
import sys
import time
import xmlrpc.client


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', 0))
        return s.getsockname()[1]


def wait_for_master(proxy, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            code, _, _ = proxy.getPid('/probe')
            if code == 1:
                return
        except Exception:
            pass
        time.sleep(0.05)
    raise RuntimeError("Master did not start within %.1fs" % timeout)


def find_python_master():
    import shutil
    path = shutil.which('rosmaster')
    if path:
        return path
    raise RuntimeError("Cannot find 'rosmaster' on PATH")


def find_cpp_master():
    import roslib.packages
    matches = roslib.packages.find_node('rosmaster', 'rosmaster_cpp')
    if matches:
        return matches[0]
    raise RuntimeError("Cannot find 'rosmaster_cpp'")


class MasterProcess:
    def __init__(self, binary, port):
        self.port = port
        self.proc = subprocess.Popen(
            [binary, '--core', '-p', str(port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.proxy = xmlrpc.client.ServerProxy('http://localhost:%d/' % port)
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
# Population & Benchmarking
# ---------------------------------------------------------------------------

def populate(proxy, num_robots, topics_per_robot, services_per_robot):
    """Register a simulated fleet of robots."""
    for r in range(num_robots):
        node = '/robot_%04d/autonomy' % r
        api = 'http://localhost:%d/' % (20000 + r)

        for t in range(topics_per_robot):
            topic = '/robot_%04d/topic_%03d' % (r, t)
            proxy.registerPublisher(node, topic, 'std_msgs/String', api)

        for s in range(services_per_robot):
            svc = '/robot_%04d/service_%03d' % (r, s)
            proxy.registerService(node, svc, 'rosrpc://localhost:%d' % (30000 + r), api)

    # Also register a few subscribers to make getSystemState richer
    for r in range(num_robots):
        sub_node = '/robot_%04d/navigator' % r
        sub_api = 'http://localhost:%d/' % (40000 + r)
        # Each robot subscribes to a handful of topics from other robots
        for t in range(min(3, topics_per_robot)):
            other = (r + 1) % num_robots
            topic = '/robot_%04d/topic_%03d' % (other, t)
            proxy.registerSubscriber(sub_node, topic, 'std_msgs/String', sub_api)


def populate_params(proxy, num_robots, params_per_robot):
    """Set parameters simulating robot configuration."""
    for r in range(num_robots):
        for p in range(params_per_robot):
            key = '/robot_%04d/param_%03d' % (r, p)
            proxy.setParam('/config', key, 'value_%d_%d' % (r, p))


def benchmark_call(proxy, method, args, iterations=20):
    """Time repeated XML-RPC calls. Returns list of durations in ms."""
    func = getattr(proxy, method)
    # Warm up
    for _ in range(3):
        func(*args)

    times = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        func(*args)
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
    return times


def run_benchmarks(proxy, label, iterations=20):
    """Run all benchmark scenarios and return results dict."""
    results = {}

    # --- rostopic list: getSystemState + getTopicTypes ---
    times = benchmark_call(proxy, 'getSystemState', ('/bench',), iterations)
    results['getSystemState'] = times

    times = benchmark_call(proxy, 'getTopicTypes', ('/bench',), iterations)
    results['getTopicTypes'] = times

    # Combined: simulates `rostopic list`
    combined = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        proxy.getSystemState('/bench')
        proxy.getTopicTypes('/bench')
        t1 = time.perf_counter()
        combined.append((t1 - t0) * 1000.0)
    results['rostopic_list (combined)'] = combined

    # --- rosservice list: getSystemState (same call, included for clarity) ---

    # --- Parameter server ---
    times = benchmark_call(proxy, 'getParamNames', ('/bench',), iterations)
    results['getParamNames'] = times

    # --- getPublishedTopics ---
    times = benchmark_call(proxy, 'getPublishedTopics', ('/bench', ''), iterations)
    results['getPublishedTopics'] = times

    # --- Single lookups (constant-time sanity check) ---
    times = benchmark_call(proxy, 'lookupNode', ('/bench', '/robot_0000/autonomy'), iterations)
    results['lookupNode'] = times

    times = benchmark_call(proxy, 'lookupService', ('/bench', '/robot_0000/service_000'), iterations)
    results['lookupService'] = times

    return results


def run_registration_benchmarks(binary, num_registrations, iterations=5):
    """Benchmark bulk registration throughput on a fresh master per iteration.

    Simulates a multimaster_fkie sync burst: many registerPublisher,
    registerSubscriber, and registerService calls in rapid succession.
    Returns dict of operation -> list of (total_ms, ops_per_sec).
    """
    results = {
        'registerPublisher': [],
        'registerSubscriber': [],
        'registerService': [],
        'mixed sync burst': [],
    }

    for _ in range(iterations):
        port = find_free_port()
        master = MasterProcess(binary, port)
        proxy = master.proxy

        # registerPublisher burst
        t0 = time.perf_counter()
        for i in range(num_registrations):
            proxy.registerPublisher(
                '/node_%04d' % i, '/topic_%04d' % i,
                'std_msgs/String', 'http://localhost:%d/' % (20000 + i))
        elapsed = (time.perf_counter() - t0) * 1000.0
        results['registerPublisher'].append(elapsed)

        master.stop()

        # registerSubscriber burst
        port = find_free_port()
        master = MasterProcess(binary, port)
        proxy = master.proxy

        t0 = time.perf_counter()
        for i in range(num_registrations):
            proxy.registerSubscriber(
                '/node_%04d' % i, '/topic_%04d' % i,
                'std_msgs/String', 'http://localhost:%d/' % (20000 + i))
        elapsed = (time.perf_counter() - t0) * 1000.0
        results['registerSubscriber'].append(elapsed)

        master.stop()

        # registerService burst
        port = find_free_port()
        master = MasterProcess(binary, port)
        proxy = master.proxy

        t0 = time.perf_counter()
        for i in range(num_registrations):
            proxy.registerService(
                '/node_%04d' % i, '/svc_%04d' % i,
                'rosrpc://localhost:%d' % (30000 + i),
                'http://localhost:%d/' % (20000 + i))
        elapsed = (time.perf_counter() - t0) * 1000.0
        results['registerService'].append(elapsed)

        master.stop()

        # Mixed burst: simulates a real sync event
        # (pubs + subs + services interleaved, like multimaster_fkie would do)
        port = find_free_port()
        master = MasterProcess(binary, port)
        proxy = master.proxy

        count = num_registrations // 3
        t0 = time.perf_counter()
        for i in range(count):
            proxy.registerPublisher(
                '/sync_%04d' % i, '/topic_%04d' % i,
                'std_msgs/String', 'http://localhost:%d/' % (20000 + i))
            proxy.registerSubscriber(
                '/sync_%04d' % i, '/sub_topic_%04d' % i,
                'std_msgs/String', 'http://localhost:%d/' % (20000 + i))
            proxy.registerService(
                '/sync_%04d' % i, '/svc_%04d' % i,
                'rosrpc://localhost:%d' % (30000 + i),
                'http://localhost:%d/' % (20000 + i))
        elapsed = (time.perf_counter() - t0) * 1000.0
        results['mixed sync burst'].append(elapsed)

        master.stop()

    return results


def print_registration_results(py_reg, cpp_reg, num_registrations):
    print("\n" + "=" * 82)
    print("  REGISTRATION THROUGHPUT BENCHMARK  (%d registrations per burst)" % num_registrations)
    print("=" * 82)
    print("  %-25s  %16s  %16s  %9s" % ("Operation", "Python (ms)", "C++ (ms)", "Speedup"))
    print("  " + "-" * 78)

    for op in py_reg:
        py_med, py_sd = fmt_stats(py_reg[op])
        cpp_med, cpp_sd = fmt_stats(cpp_reg[op])
        speedup = py_med / cpp_med if cpp_med > 0 else float('inf')
        n = num_registrations if 'mixed' not in op else (num_registrations // 3) * 3
        py_ops = n / (py_med / 1000.0)
        cpp_ops = n / (cpp_med / 1000.0)
        print("  %-25s  %6.1f ± %6.1f  %6.1f ± %6.1f  %7.1fx"
              % (op, py_med, py_sd, cpp_med, cpp_sd, speedup))
        print("  %25s  %10.0f op/s  %10.0f op/s" % ('', py_ops, cpp_ops))

    print("=" * 82)
    print()


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def fmt_stats(times):
    """Format timing stats: median ± stdev."""
    med = statistics.median(times)
    sd = statistics.stdev(times) if len(times) > 1 else 0.0
    return med, sd


def print_results(py_results, cpp_results, num_topics, num_services, num_params):
    total = num_topics + num_services + num_params
    print("\n" + "=" * 82)
    print("  ROSMASTER PERFORMANCE BENCHMARK")
    print("  Topics: %d  |  Services: %d  |  Params: %d  |  Total registrations: %d"
          % (num_topics, num_services, num_params, total))
    print("=" * 82)
    print("  %-30s  %14s  %14s  %9s" % ("Method", "Python (ms)", "C++ (ms)", "Speedup"))
    print("  " + "-" * 78)

    for method in py_results:
        py_med, py_sd = fmt_stats(py_results[method])
        cpp_med, cpp_sd = fmt_stats(cpp_results[method])
        speedup = py_med / cpp_med if cpp_med > 0 else float('inf')
        print("  %-30s  %6.2f ± %5.2f  %6.2f ± %5.2f  %7.1fx"
              % (method, py_med, py_sd, cpp_med, cpp_sd, speedup))

    print("=" * 82)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Rosmaster performance benchmark')
    parser.add_argument('--robots', type=int, default=100,
                        help='Number of simulated robots (default: 100)')
    parser.add_argument('--topics-per-robot', type=int, default=20,
                        help='Topics per robot (default: 20)')
    parser.add_argument('--services-per-robot', type=int, default=5,
                        help='Services per robot (default: 5)')
    parser.add_argument('--params-per-robot', type=int, default=10,
                        help='Params per robot (default: 10)')
    parser.add_argument('--iterations', type=int, default=20,
                        help='Iterations per benchmark call (default: 20)')
    parser.add_argument('--reg-burst', type=int, default=1000,
                        help='Registrations per burst in throughput test (default: 1000)')
    parser.add_argument('--reg-iterations', type=int, default=5,
                        help='Iterations for registration throughput test (default: 5)')
    args = parser.parse_args()

    num_topics = args.robots * args.topics_per_robot
    num_services = args.robots * args.services_per_robot
    num_params = args.robots * args.params_per_robot

    print("Benchmark config: %d robots × (%d topics + %d services + %d params) = %d total"
          % (args.robots, args.topics_per_robot, args.services_per_robot,
             args.params_per_robot, num_topics + num_services + num_params))

    # --- Python master ---
    print("\n[1/4] Starting Python master...")
    py_port = find_free_port()
    py_master = MasterProcess(find_python_master(), py_port)

    print("[2/4] Populating Python master (%d topics, %d services, %d params)..."
          % (num_topics, num_services, num_params))
    populate(py_master.proxy, args.robots, args.topics_per_robot, args.services_per_robot)
    populate_params(py_master.proxy, args.robots, args.params_per_robot)
    print("       Running benchmarks...")
    py_results = run_benchmarks(py_master.proxy, 'Python', args.iterations)
    py_master.stop()

    # --- C++ master ---
    print("[3/4] Starting C++ master...")
    cpp_port = find_free_port()
    cpp_master = MasterProcess(find_cpp_master(), cpp_port)

    print("[4/4] Populating C++ master (%d topics, %d services, %d params)..."
          % (num_topics, num_services, num_params))
    populate(cpp_master.proxy, args.robots, args.topics_per_robot, args.services_per_robot)
    populate_params(cpp_master.proxy, args.robots, args.params_per_robot)
    print("       Running benchmarks...")
    cpp_results = run_benchmarks(cpp_master.proxy, 'C++', args.iterations)
    cpp_master.stop()

    # --- Report ---
    print_results(py_results, cpp_results, num_topics, num_services, num_params)

    # --- Registration throughput ---
    py_binary = find_python_master()
    cpp_binary = find_cpp_master()

    print("[5/6] Registration throughput: Python master (%d registrations × %d iterations)..."
          % (args.reg_burst, args.reg_iterations))
    py_reg = run_registration_benchmarks(py_binary, args.reg_burst, args.reg_iterations)

    print("[6/6] Registration throughput: C++ master (%d registrations × %d iterations)..."
          % (args.reg_burst, args.reg_iterations))
    cpp_reg = run_registration_benchmarks(cpp_binary, args.reg_burst, args.reg_iterations)

    print_registration_results(py_reg, cpp_reg, args.reg_burst)


if __name__ == '__main__':
    main()
