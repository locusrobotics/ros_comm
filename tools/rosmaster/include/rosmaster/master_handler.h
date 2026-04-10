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

#ifndef ROSMASTER_MASTER_HANDLER_H
#define ROSMASTER_MASTER_HANDLER_H

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "xmlrpcpp/XmlRpcServer.h"
#include "xmlrpcpp/XmlRpcServerMethod.h"
#include "xmlrpcpp/XmlRpcValue.h"

#include "rosmaster/param_server.h"
#include "rosmaster/registrations.h"
#include "rosmaster/thread_pool.h"

namespace rosmaster
{

/// Log levels matching Python's logging module
enum class LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

/// Set the global rosmaster log level
void setLogLevel(LogLevel level);

/// XML-RPC handler for the ROS Master API.
/// Implements all 22 master API methods as XmlRpcServerMethods.
class ROSMasterHandler
{
public:
  static constexpr int NUM_WORKERS = 3;

  explicit ROSMasterHandler(XmlRpc::XmlRpcServer* server, int num_workers = NUM_WORKERS);
  ~ROSMasterHandler();

  /// Set the master's URI (called when server is ready)
  void setUri(const std::string& uri);

  /// Check if the master is still running
  bool ok() const;

  /// Shut down the master
  void shutdown(const std::string& reason = "");

private:
  // Helper: build a standard (code, msg, val) response array
  static void setResponse(XmlRpc::XmlRpcValue& result, int code, const std::string& msg,
                           const XmlRpc::XmlRpcValue& val);

  // Helper: parse host:port from an http:// URI
  static bool parseUri(const std::string& uri, std::string& host, int& port, std::string& path);

  // Helper: make an XML-RPC call to a node
  static bool callNode(const std::string& uri, const std::string& method, XmlRpc::XmlRpcValue& params,
                        XmlRpc::XmlRpcValue& result);

  // Notification helpers
  void notifyTopicSubscribers(const std::string& topic, const std::vector<std::string>& pub_uris,
                               const std::vector<std::string>& sub_uris);
  void notifyParamSubscribers(const std::vector<ParamUpdate>& updates);
  void publisherUpdateTask(const std::string& api, const std::string& topic,
                            const std::vector<std::string>& pub_uris);
  void paramUpdateTask(const std::string& caller_id, const std::string& caller_api,
                        const std::string& param_key, const XmlRpc::XmlRpcValue& param_value);

  // Validate caller_id is a string. Returns false and sets error response if invalid.
  bool validateCallerId(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result, int expected_args,
                         const XmlRpc::XmlRpcValue& error_val);

  // --- API method implementations ---
  void handleShutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetUri(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetPid(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  // Parameter server
  void handleDeleteParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleSetParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleSearchParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleSubscribeParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleUnsubscribeParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleHasParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetParamNames(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  // Services
  void handleRegisterService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleLookupService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleUnregisterService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  // Pub/Sub
  void handleRegisterSubscriber(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleUnregisterSubscriber(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleRegisterPublisher(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleUnregisterPublisher(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  // Graph state
  void handleLookupNode(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetPublishedTopics(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetTopicTypes(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void handleGetSystemState(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  // --- Data members ---
  std::string uri_;
  std::atomic<bool> done_{false};

  std::unique_ptr<MarkedThreadPool> thread_pool_;
  std::mutex ps_lock_;  // protects pub/sub/param operations

  RegistrationManager reg_manager_;
  std::unordered_map<std::string, std::string> topics_types_;  // { topicName: type }

  ParamDictionary param_server_;
  XmlRpc::XmlRpcServer* server_;

  // Store method objects (must outlive server)
  std::vector<std::unique_ptr<XmlRpc::XmlRpcServerMethod>> methods_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_MASTER_HANDLER_H
