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

#ifndef ROSMASTER_MASTER_H
#define ROSMASTER_MASTER_H

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "xmlrpcpp/XmlRpcServer.h"

namespace rosmaster
{

class ROSMasterHandler;

/// Default TCP port for the ROS master XML-RPC server.
constexpr int DEFAULT_MASTER_PORT = 11311;

/**
 * @brief High-level ROS master server wrapper.
 *
 * Owns and manages the XML-RPC server and master request handler lifecycle,
 * including startup, shutdown, and periodic request processing.
 */
class Master
{
public:
  /**
   * @brief Construct a master instance.
   * @param port TCP port to bind the XML-RPC server to.
   * @param num_workers Number of worker threads used for asynchronous tasks.
   */
  explicit Master(int port = DEFAULT_MASTER_PORT, int num_workers = 3);

  /**
   * @brief Destroy the master instance.
   */
  ~Master();

  /**
   * @brief Start the master server.
   *
   * Initializes the XML-RPC server and handler, then blocks until the server
   * is ready to accept requests.
   */
  void start();

  /**
   * @brief Check whether the master is currently running.
   * @return True if the server is running, false otherwise.
   */
  bool ok() const;

  /**
   * @brief Stop the master server and release resources.
   */
  void stop();

  /**
   * @brief Get the master's XML-RPC URI.
   * @return Fully qualified XML-RPC URI for this master instance.
   */
  const std::string& uri() const;

  /**
   * @brief Process pending XML-RPC requests once.
   *
   * Intended to be called repeatedly from the main loop.
   * @param timeout_ms Maximum time in milliseconds to wait for work.
   */
  void spinOnce(double timeout_ms = 100.0);

  /**
   * @brief Interrupt a currently blocked spinOnce call.
   *
   * Useful for waking the main loop so it can re-check shutdown or other
   * control conditions.
   */
  void interrupt();

private:
  int port_;
  int num_workers_;

  std::string uri_;

  std::unique_ptr<XmlRpc::XmlRpcServer> server_;

  std::unique_ptr<ROSMasterHandler> handler_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_MASTER_H
