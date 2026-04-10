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

/// Default master port
constexpr int DEFAULT_MASTER_PORT = 11311;

/// High-level master server wrapper.
/// Creates an XmlRpcServer, registers the master API, and manages the server lifecycle.
class Master
{
public:
  explicit Master(int port = DEFAULT_MASTER_PORT, int num_workers = 3);
  ~Master();

  /// Start the master. Blocks until server is ready.
  void start();

  /// Check if the master is still running
  bool ok() const;

  /// Stop the master
  void stop();

  /// Get the master's XML-RPC URI
  std::string uri() const;

  /// Process requests (call in a loop from main thread)
  void spinOnce(double timeout_ms = 100.0);

  /// Interrupt the current spinOnce so the main loop can re-check conditions
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
