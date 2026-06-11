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

#include <atomic>
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

/**
 * @brief Log levels matching the Python rosmaster's logging module semantics.
 */
enum class LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

/**
 * @brief Set the global rosmaster log verbosity threshold.
 * @param level Minimum severity level to emit.
 */
void setLogLevel(LogLevel level);

/**
 * @brief XML-RPC handler that implements the ROS Master API.
 *
 * Registers and serves ROS master XML-RPC methods, manages graph registrations, tracks parameter state, and dispatches
 * notifications to affected nodes.
 */
class ROSMasterHandler
{
public:
  /// Default number of worker threads for asynchronous node callback notifications.
  static constexpr int NUM_WORKERS = 3;

  /**
   * @brief Construct a master API handler and register server methods.
   * @param server XML-RPC server used to expose master methods.
   * @param num_workers Number of worker threads for asynchronous callbacks.
   */
  explicit ROSMasterHandler(XmlRpc::XmlRpcServer* server, int num_workers = NUM_WORKERS);

  /**
   * @brief Destroy the handler and release owned resources.
   */
  ~ROSMasterHandler();

  /**
   * @brief Set the master XML-RPC URI once the server is ready.
   * @param uri Fully qualified URI for this master instance.
   */
  void setUri(const std::string& uri);

  /**
   * @brief Check whether shutdown has been requested.
   * @return True when the master should continue running, false when done.
   */
  bool ok() const;

  /**
   * @brief Request master shutdown.
   * @param reason Optional human-readable shutdown reason.
   */
  void shutdown(const std::string& reason = "");

private:
  /**
   * @brief Build a standard ROS master XML-RPC response tuple.
   * @param result Output XML-RPC array to populate.
   * @param code Status code.
   * @param msg Human-readable status message.
   * @param val API-specific response payload.
   */
  static void setResponse(
    XmlRpc::XmlRpcValue& result,
    int code,
    const std::string& msg,
    const XmlRpc::XmlRpcValue& val);

  /**
   * @brief Parse an HTTP URI into host, port, and path components.
   * @param uri Input URI string.
   * @param host Parsed host output.
   * @param port Parsed port output.
   * @param path Parsed path output.
   * @return True if parsing succeeds and a usable host/port is found.
   */
  static bool parseUri(const std::string& uri, std::string& host, int& port, std::string& path);

  /**
   * @brief Invoke a method on a node's XML-RPC endpoint.
   * @param uri Target node URI.
   * @param method Method name to invoke.
   * @param params Method parameter array.
   * @param result Output XML-RPC result value.
   * @return True if the XML-RPC call transport succeeds.
   */
  static bool callNode(
    const std::string& uri,
    const std::string& method,
    XmlRpc::XmlRpcValue& params,
    XmlRpc::XmlRpcValue& result);

  /**
   * @brief Notify subscribers of a topic about updated publisher URIs.
   * @param topic Topic name being updated.
   * @param pub_uris Current publisher XML-RPC URIs for the topic.
   * @param sub_uris Subscriber XML-RPC URIs to notify.
   */
  void notifyTopicSubscribers(
    const std::string& topic,
    const std::vector<std::string>& pub_uris,
    const std::vector<std::string>& sub_uris);

  /**
   * @brief Notify parameter subscribers about parameter value changes.
   * @param updates Batched parameter updates to deliver.
   */
  void notifyParamSubscribers(const std::vector<ParamUpdate>& updates);

  /**
   * @brief Background task to send a publisherUpdate callback to one node.
   * @param api Subscriber XML-RPC URI.
   * @param topic Topic name.
   * @param pub_uris Current publisher XML-RPC URIs.
   */
  void publisherUpdateTask(
    const std::string& api,
    const std::string& topic,
    const std::vector<std::string>& pub_uris);

  /**
   * @brief Background task to send a paramUpdate callback to one node.
   * @param caller_id Subscriber node name.
   * @param caller_api Subscriber XML-RPC URI.
   * @param param_key Parameter key that changed.
   * @param param_value Updated parameter value.
   */
  void paramUpdateTask(
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& param_key,
    const XmlRpc::XmlRpcValue& param_value);

  /**
   * @brief Validate a request's caller_id argument and expected arity.
   * @param params Incoming XML-RPC parameter array.
   * @param result Output response populated on validation failure.
   * @param expected_args Expected number of request arguments.
   * @param error_val Fallback value to return in failure responses.
   * @return True when validation succeeds, false otherwise.
   */
  bool validateCallerId(
    XmlRpc::XmlRpcValue& params,
    XmlRpc::XmlRpcValue& result,
    int expected_args,
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

  // Master XML-RPC URI returned by getUri and related responses.
  std::string uri_;

  // Shutdown flag set when master termination is requested.
  std::atomic<bool> done_{false};

  // Thread pool used for asynchronous node callback notifications.
  std::unique_ptr<MarkedThreadPool> thread_pool_;

  // Mutex protecting registration and parameter server state mutations.
  std::mutex ps_lock_;

  // Publisher, subscriber, service, and param subscriber registration tables.
  RegistrationManager reg_manager_;

  // Topic type map keyed by topic name.
  std::unordered_map<std::string, std::string> topics_types_;

  // In-memory parameter storage and query/update engine.
  ParamDictionary param_server_;

  // Non-owning pointer to the XML-RPC server used for method registration.
  XmlRpc::XmlRpcServer* server_;

  // Owns registered XML-RPC method objects, which must outlive server hooks.
  std::vector<std::unique_ptr<XmlRpc::XmlRpcServerMethod>> methods_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_MASTER_HANDLER_H
