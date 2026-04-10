// Software License Agreement (BSD License)
//
// Copyright (c) 2025, Locus Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Locus Robotics nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * xmlrpcpp serialization micro-benchmark
 *
 * Builds large XmlRpcValue structures that mirror real ROS master responses
 * (getSystemState, getTopicTypes, etc.) and times toXml() and fromXml().
 *
 * Usage:
 *   rosrun xmlrpcpp xmlrpc_bench [--topics N] [--services N]
 *
 * Build:
 *   Added to xmlrpcpp/CMakeLists.txt as a test/benchmark target.
 */

#include "xmlrpcpp/XmlRpcValue.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using XmlRpc::XmlRpcValue;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct BenchResult
{
  std::string name;
  double median_ms;
  double stdev_ms;
  size_t xml_bytes;
};

std::vector<double> run_timed(int iterations, std::function<void()> fn)
{
  // Warm up
  for (int i = 0; i < 2; ++i)
    fn();

  std::vector<double> times;
  times.reserve(iterations);
  for (int i = 0; i < iterations; ++i)
  {
    auto t0 = std::chrono::high_resolution_clock::now();
    fn();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    times.push_back(ms);
  }
  return times;
}

double median(std::vector<double>& v)
{
  size_t n = v.size();
  std::sort(v.begin(), v.end());
  if (n % 2 == 0)
    return (v[n / 2 - 1] + v[n / 2]) / 2.0;
  return v[n / 2];
}

double stdev(const std::vector<double>& v, double mean)
{
  double sum = 0;
  for (double x : v)
    sum += (x - mean) * (x - mean);
  return std::sqrt(sum / v.size());
}

// ---------------------------------------------------------------------------
// Build realistic XmlRpcValue structures
// ---------------------------------------------------------------------------

// Mirrors getSystemState response: [[pub_topic, [nodes]], ...] × 3
XmlRpcValue build_system_state(int num_topics, int num_services)
{
  // Publishers: [["/topic_0000", ["/node_0000"]], ...]
  XmlRpcValue pubs;
  pubs.setSize(0);
  for (int i = 0; i < num_topics; ++i)
  {
    XmlRpcValue entry;
    entry.setSize(2);
    entry[0] = "/topic_" + std::to_string(i);
    XmlRpcValue nodes;
    nodes.setSize(1);
    nodes[0] = "/robot_" + std::to_string(i / 20) + "/autonomy";
    entry[1] = nodes;
    pubs[pubs.size()] = entry;
  }

  // Subscribers: same structure, fewer entries
  XmlRpcValue subs;
  subs.setSize(0);
  int num_subs = num_topics / 5;
  for (int i = 0; i < num_subs; ++i)
  {
    XmlRpcValue entry;
    entry.setSize(2);
    entry[0] = "/topic_" + std::to_string(i);
    XmlRpcValue nodes;
    nodes.setSize(1);
    nodes[0] = "/robot_" + std::to_string(i / 20) + "/navigator";
    entry[1] = nodes;
    subs[subs.size()] = entry;
  }

  // Services
  XmlRpcValue svcs;
  svcs.setSize(0);
  for (int i = 0; i < num_services; ++i)
  {
    XmlRpcValue entry;
    entry.setSize(2);
    entry[0] = "/robot_" + std::to_string(i / 5) + "/service_" + std::to_string(i % 5);
    XmlRpcValue nodes;
    nodes.setSize(1);
    nodes[0] = "/robot_" + std::to_string(i / 5) + "/autonomy";
    entry[1] = nodes;
    svcs[svcs.size()] = entry;
  }

  // Full response: [1, "ok", [[pubs], [subs], [svcs]]]
  XmlRpcValue state;
  state.setSize(3);
  state[0] = pubs;
  state[1] = subs;
  state[2] = svcs;

  XmlRpcValue result;
  result.setSize(3);
  result[0] = 1;
  result[1] = std::string("current system state");
  result[2] = state;
  return result;
}

// Mirrors getTopicTypes response: [[topic, type], ...]
XmlRpcValue build_topic_types(int num_topics)
{
  XmlRpcValue types;
  types.setSize(0);
  for (int i = 0; i < num_topics; ++i)
  {
    XmlRpcValue entry;
    entry.setSize(2);
    entry[0] = "/topic_" + std::to_string(i);
    entry[1] = "std_msgs/String";
    types[types.size()] = entry;
  }

  XmlRpcValue result;
  result.setSize(3);
  result[0] = 1;
  result[1] = std::string("current topics");
  result[2] = types;
  return result;
}

// Mirrors getParamNames response: [1, "ok", ["/param_0", ...]]
XmlRpcValue build_param_names(int num_params)
{
  XmlRpcValue names;
  names.setSize(num_params);
  for (int i = 0; i < num_params; ++i)
  {
    names[i] = "/robot_" + std::to_string(i / 10) + "/param_" + std::to_string(i % 10);
  }

  XmlRpcValue result;
  result.setSize(3);
  result[0] = 1;
  result[1] = std::string("parameter names");
  result[2] = names;
  return result;
}

// ---------------------------------------------------------------------------
// Benchmark runner
// ---------------------------------------------------------------------------

BenchResult bench_toXml(const std::string& name, const XmlRpcValue& value, int iterations)
{
  std::string xml;
  auto times = run_timed(iterations, [&]() { xml = value.toXml(); });

  double med = median(times);
  double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
  double sd = stdev(times, mean);

  return {name, med, sd, xml.size()};
}

BenchResult bench_fromXml(const std::string& name, const std::string& xml, int iterations)
{
  auto times = run_timed(iterations, [&]() {
    int offset = 0;
    XmlRpcValue parsed(xml, &offset);
  });

  double med = median(times);
  double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
  double sd = stdev(times, mean);

  return {name, med, sd, xml.size()};
}

void print_results(const std::vector<BenchResult>& results)
{
  std::cout << "\n";
  std::cout << std::string(78, '=') << "\n";
  std::cout << "  XMLRPCPP SERIALIZATION MICRO-BENCHMARK\n";
  std::cout << std::string(78, '=') << "\n";
  std::cout << "  " << std::left << std::setw(35) << "Operation"
            << std::right << std::setw(14) << "Time (ms)"
            << std::setw(14) << "XML size"
            << std::setw(12) << "MB/s" << "\n";
  std::cout << "  " << std::string(74, '-') << "\n";

  for (const auto& r : results)
  {
    double mbps = (r.xml_bytes / (1024.0 * 1024.0)) / (r.median_ms / 1000.0);
    std::cout << "  " << std::left << std::setw(35) << r.name
              << std::right << std::fixed << std::setprecision(2)
              << std::setw(7) << r.median_ms << " ± "
              << std::setw(4) << r.stdev_ms
              << std::setw(10) << (r.xml_bytes / 1024) << " KB"
              << std::setw(10) << std::setprecision(1) << mbps << "\n";
  }

  std::cout << std::string(78, '=') << "\n\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  int num_topics = 2000;
  int num_services = 500;
  int num_params = 1000;
  int iterations = 20;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--topics" && i + 1 < argc)
      num_topics = std::atoi(argv[++i]);
    else if (arg == "--services" && i + 1 < argc)
      num_services = std::atoi(argv[++i]);
    else if (arg == "--params" && i + 1 < argc)
      num_params = std::atoi(argv[++i]);
    else if (arg == "--iterations" && i + 1 < argc)
      iterations = std::atoi(argv[++i]);
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0]
                << " [--topics N] [--services N] [--params N] [--iterations N]\n";
      return 0;
    }
  }

  std::cout << "Building test data: " << num_topics << " topics, "
            << num_services << " services, " << num_params << " params\n";

  // Build data structures
  auto system_state = build_system_state(num_topics, num_services);
  auto topic_types = build_topic_types(num_topics);
  auto param_names = build_param_names(num_params);

  // Pre-serialize for fromXml benchmarks
  std::string system_state_xml = system_state.toXml();
  std::string topic_types_xml = topic_types.toXml();
  std::string param_names_xml = param_names.toXml();

  std::cout << "XML sizes: getSystemState=" << (system_state_xml.size() / 1024) << " KB"
            << "  getTopicTypes=" << (topic_types_xml.size() / 1024) << " KB"
            << "  getParamNames=" << (param_names_xml.size() / 1024) << " KB\n";

  // Run benchmarks
  std::vector<BenchResult> results;

  std::cout << "\nRunning toXml benchmarks (" << iterations << " iterations)...\n";
  results.push_back(bench_toXml("toXml: getSystemState", system_state, iterations));
  results.push_back(bench_toXml("toXml: getTopicTypes", topic_types, iterations));
  results.push_back(bench_toXml("toXml: getParamNames", param_names, iterations));

  std::cout << "Running fromXml benchmarks (" << iterations << " iterations)...\n";
  results.push_back(bench_fromXml("fromXml: getSystemState", system_state_xml, iterations));
  results.push_back(bench_fromXml("fromXml: getTopicTypes", topic_types_xml, iterations));
  results.push_back(bench_fromXml("fromXml: getParamNames", param_names_xml, iterations));

  print_results(results);

  // Also run at larger scale
  if (num_topics <= 2000)
  {
    std::cout << "--- Scaling test: 15000 topics ---\n";
    auto big_state = build_system_state(15000, 5000);
    auto big_types = build_topic_types(15000);
    std::string big_state_xml = big_state.toXml();
    std::string big_types_xml = big_types.toXml();

    std::vector<BenchResult> scale_results;
    scale_results.push_back(bench_toXml("toXml: 15K getSystemState", big_state, 10));
    scale_results.push_back(bench_toXml("toXml: 15K getTopicTypes", big_types, 10));
    scale_results.push_back(bench_fromXml("fromXml: 15K getSystemState", big_state_xml, 10));
    scale_results.push_back(bench_fromXml("fromXml: 15K getTopicTypes", big_types_xml, 10));
    print_results(scale_results);
  }

  return 0;
}
