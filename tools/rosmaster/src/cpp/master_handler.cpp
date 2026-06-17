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

#include "rosmaster/master_handler.h"
#include "rosmaster/names.h"
#include "rosmaster/validators.h"

#include "xmlrpcpp/XmlRpcClient.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>

#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif

namespace rosmaster
{

namespace
{

LogLevel g_log_level = LogLevel::WARN;

/**
 * @class MasterMethod
 * @brief Lightweight XmlRpcServerMethod that delegates to a std::function
 */
class MasterMethod : public XmlRpc::XmlRpcServerMethod
{
public:
  using Handler = std::function<void(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&)>;

  MasterMethod(const std::string& name, Handler handler, XmlRpc::XmlRpcServer* server)
    : XmlRpcServerMethod(name, server),
      handler_(std::move(handler))
  {
  }

  void execute(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) override
  {
    handler_(params, result);
  }

private:
  Handler handler_;
};

void mloginfo(const std::string& msg)
{
  if (g_log_level <= LogLevel::INFO)
  {
    std::cout << "[INFO] [rosmaster]: " << msg << std::endl;
  }
}

void mlogwarn(const std::string& msg)
{
  if (g_log_level <= LogLevel::WARN)
  {
    std::cerr << "[WARN] [rosmaster]: " << msg << std::endl;
  }
}

}  // anonymous namespace

void setLogLevel(LogLevel level)
{
  g_log_level = level;
}

// --- Static helpers ---

void ROSMasterHandler::setResponse(
  XmlRpc::XmlRpcValue& result,
  int code,
  const std::string& msg,
  const XmlRpc::XmlRpcValue& val)
{
  result.setSize(3);
  result[0] = code;
  result[1] = msg;
  result[2] = val;
}

bool ROSMasterHandler::parseUri(
  const std::string& uri,
  std::string& host,
  int& port,
  std::string& path)
{
  if (uri.substr(0, 7) != "http://")
  {
    return false;
  }
  auto rest = uri.substr(7);
  auto slash_pos = rest.find('/');
  std::string host_port = (slash_pos != std::string::npos) ? rest.substr(0, slash_pos) : rest;
  path = (slash_pos != std::string::npos) ? rest.substr(slash_pos) : "/";
  auto colon_pos = host_port.rfind(':');
  if (colon_pos == std::string::npos)
  {
    return false;
  }
  host = host_port.substr(0, colon_pos);
  try
  {
    port = std::stoi(host_port.substr(colon_pos + 1));
  }
  catch (...)
  {
    return false;
  }
  return !host.empty() && port > 0;
}

bool ROSMasterHandler::callNode(
  const std::string& uri,
  const std::string& method,
  XmlRpc::XmlRpcValue& params,
  XmlRpc::XmlRpcValue& result)
{
  std::string host, path;
  int port = 0;
  if (!parseUri(uri, host, port, path))
  {
    return false;
  }
  try
  {
    XmlRpc::XmlRpcClient client(host.c_str(), port, path.c_str());
    return client.execute(method.c_str(), params, result);
  }
  catch (...)
  {
    return false;
  }
}

bool ROSMasterHandler::validateCallerId(
  XmlRpc::XmlRpcValue& params,
  XmlRpc::XmlRpcValue& result,
  int expected_args,
  const XmlRpc::XmlRpcValue& error_val)
{
  if (params.size() < 1)
  {
    setResponse(result, -1, "missing required caller_id parameter", error_val);
    return false;
  }
  if (params.size() != expected_args)
  {
    setResponse(result, -1, "Error: bad call arity", error_val);
    return false;
  }
  if (params[0].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    setResponse(result, -1, "caller_id must be a string", error_val);
    return false;
  }
  return true;
}

// --- Constructor / Destructor ---

ROSMasterHandler::ROSMasterHandler(XmlRpc::XmlRpcServer* server, int num_workers)
  : thread_pool_(std::make_unique<MarkedThreadPool>(num_workers))
  , reg_manager_(thread_pool_.get())
  , param_server_(&reg_manager_)
  , server_(server)
{
  // Register all 22 XML-RPC methods
  auto reg = [this](const std::string& name, MasterMethod::Handler handler) {
    auto m = std::make_unique<MasterMethod>(name, std::move(handler), server_);
    methods_.push_back(std::move(m));
  };

  // System
  reg("shutdown", [this](auto& p, auto& r) { handleShutdown(p, r); });
  reg("getUri", [this](auto& p, auto& r) { handleGetUri(p, r); });
  reg("getPid", [this](auto& p, auto& r) { handleGetPid(p, r); });

  // Parameter Server
  reg("deleteParam", [this](auto& p, auto& r) { handleDeleteParam(p, r); });
  reg("setParam", [this](auto& p, auto& r) { handleSetParam(p, r); });
  reg("getParam", [this](auto& p, auto& r) { handleGetParam(p, r); });
  reg("searchParam", [this](auto& p, auto& r) { handleSearchParam(p, r); });
  reg("subscribeParam", [this](auto& p, auto& r) { handleSubscribeParam(p, r); });
  reg("unsubscribeParam", [this](auto& p, auto& r) { handleUnsubscribeParam(p, r); });
  reg("hasParam", [this](auto& p, auto& r) { handleHasParam(p, r); });
  reg("getParamNames", [this](auto& p, auto& r) { handleGetParamNames(p, r); });

  // Services
  reg("registerService", [this](auto& p, auto& r) { handleRegisterService(p, r); });
  reg("lookupService", [this](auto& p, auto& r) { handleLookupService(p, r); });
  reg("unregisterService", [this](auto& p, auto& r) { handleUnregisterService(p, r); });

  // Pub/Sub
  reg("registerSubscriber", [this](auto& p, auto& r) { handleRegisterSubscriber(p, r); });
  reg("unregisterSubscriber", [this](auto& p, auto& r) { handleUnregisterSubscriber(p, r); });
  reg("registerPublisher", [this](auto& p, auto& r) { handleRegisterPublisher(p, r); });
  reg("unregisterPublisher", [this](auto& p, auto& r) { handleUnregisterPublisher(p, r); });

  // Graph State
  reg("lookupNode", [this](auto& p, auto& r) { handleLookupNode(p, r); });
  reg("getPublishedTopics", [this](auto& p, auto& r) { handleGetPublishedTopics(p, r); });
  reg("getTopicTypes", [this](auto& p, auto& r) { handleGetTopicTypes(p, r); });
  reg("getSystemState", [this](auto& p, auto& r) { handleGetSystemState(p, r); });
}

ROSMasterHandler::~ROSMasterHandler()
{
  shutdown("destructor");
}

void ROSMasterHandler::setUri(const std::string& uri)
{
  uri_ = uri;
}

bool ROSMasterHandler::ok() const
{
  return !done_;
}

void ROSMasterHandler::shutdown(const std::string& reason)
{
  if (thread_pool_)
  {
    thread_pool_->joinAll(false);
    thread_pool_.reset();
  }
  done_ = true;
}

// --- Notification helpers ---

void ROSMasterHandler::publisherUpdateTask(const std::string& api, const std::string& topic,
                                             const std::vector<std::string>& pub_uris)
{
  mloginfo("publisherUpdate[" + topic + "] -> " + api);
  auto start = std::chrono::steady_clock::now();
  try
  {
    XmlRpc::XmlRpcValue params;
    params[0] = "/master";
    params[1] = topic;
    XmlRpc::XmlRpcValue uri_array;
    uri_array.setSize(static_cast<int>(pub_uris.size()));
    for (int i = 0; i < static_cast<int>(pub_uris.size()); ++i)
    {
      uri_array[i] = pub_uris[i];
    }
    params[2] = uri_array;
    XmlRpc::XmlRpcValue result;
    callNode(api, "publisherUpdate", params, result);
  }
  catch (const std::exception& e)
  {
    mlogwarn("publisherUpdate exception: " + std::string(e.what()));
  }
  auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  mloginfo("publisherUpdate[" + topic + "] -> " + api + ": sec=" + std::to_string(elapsed));
}

void ROSMasterHandler::paramUpdateTask(const std::string& caller_id, const std::string& caller_api,
                                         const std::string& param_key,
                                         const XmlRpc::XmlRpcValue& param_value)
{
  mloginfo("paramUpdate[" + param_key + "]");
  try
  {
    XmlRpc::XmlRpcValue params;
    params[0] = "/master";
    params[1] = param_key;
    params[2] = param_value;
    XmlRpc::XmlRpcValue result;
    if (callNode(caller_api, "paramUpdate", params, result))
    {
      // Check if node returned error code -1 (unsubscribe)
      if (result.getType() == XmlRpc::XmlRpcValue::TypeArray && result.size() >= 1 &&
          result[0].getType() == XmlRpc::XmlRpcValue::TypeInt && static_cast<int>(result[0]) == -1)
      {
        std::lock_guard<std::mutex> lock(ps_lock_);
        auto matches = reg_manager_.reverseLookup(caller_api);
        for (auto* m : matches)
        {
          param_server_.unsubscribeParam(param_key, m->id, caller_api);
        }
      }
    }
  }
  catch (...)
  {
    // Connection refused or other error - ignore
  }
}

void ROSMasterHandler::notifyTopicSubscribers(const std::string& topic,
                                                const std::vector<std::string>& pub_uris,
                                                const std::vector<std::string>& sub_uris)
{
  if (!thread_pool_)
  {
    return;
  }
  // Capture pub_uris once as shared_ptr to avoid N copies for N subscribers
  auto shared_puris = std::make_shared<std::vector<std::string>>(pub_uris);
  for (const auto& sub_api : sub_uris)
  {
    thread_pool_->queueTask(sub_api,
                             [this, sub_api, topic, shared_puris]() {
                               publisherUpdateTask(sub_api, topic, *shared_puris);
                             });
  }
}

void ROSMasterHandler::notifyParamSubscribers(const std::vector<ParamUpdate>& updates)
{
  if (!thread_pool_)
  {
    return;
  }
  for (const auto& update : updates)
  {
    // Share the param value across all subscribers for this update
    auto shared_val = std::make_shared<XmlRpc::XmlRpcValue>(update.value);
    for (const auto& [caller_id, caller_api] : update.subscribers)
    {
      thread_pool_->queueTask(
          caller_api, [this, cid = caller_id, capi = caller_api, key = update.key, shared_val]() {
            paramUpdateTask(cid, capi, key, *shared_val);
          });
    }
  }
}

// =====================================================================
// API METHOD IMPLEMENTATIONS
// =====================================================================

// --- System methods ---

void ROSMasterHandler::handleShutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string msg = params[1];
  if (!msg.empty())
  {
    std::cout << "shutdown request: " << msg << std::endl;
  }
  shutdown("external shutdown request from [" + caller_id + "]: " + msg);
  setResponse(result, 1, "shutdown", XmlRpc::XmlRpcValue(0));
}

void ROSMasterHandler::handleGetUri(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 1, XmlRpc::XmlRpcValue("")))
  {
    return;
  }
  setResponse(result, 1, "", XmlRpc::XmlRpcValue(uri_));
}

void ROSMasterHandler::handleGetPid(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 1, XmlRpc::XmlRpcValue(-1)))
  {
    return;
  }
  setResponse(result, 1, "", XmlRpc::XmlRpcValue(static_cast<int>(getpid())));
}

// --- Parameter Server methods ---

void ROSMasterHandler::handleDeleteParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string key = params[1];

  try
  {
    auto v_key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(v_key, caller_id);
    std::lock_guard<std::mutex> lock(ps_lock_);
    param_server_.deleteParam(key,
                               [this](const std::vector<ParamUpdate>& u) { notifyParamSubscribers(u); });
    mloginfo("-PARAM [" + key + "] by " + caller_id);
    setResponse(result, 1, "parameter " + key + " deleted", XmlRpc::XmlRpcValue(0));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
  catch (const std::out_of_range&)
  {
    setResponse(result, -1, "parameter [" + key + "] is not set", XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleSetParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string key = params[1];

  try
  {
    auto v_key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(v_key, caller_id);
    std::lock_guard<std::mutex> lock(ps_lock_);
    param_server_.setParam(key, params[2],
                            [this](const std::vector<ParamUpdate>& u) { notifyParamSubscribers(u); },
                            caller_id);
    mloginfo("+PARAM [" + key + "] by " + caller_id);
    setResponse(result, 1, "parameter " + key + " set", XmlRpc::XmlRpcValue(0));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
  catch (const std::exception& e)
  {
    setResponse(result, 0, std::string("Internal failure: ") + e.what(), XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleGetParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string key = params[1];

  try
  {
    auto v_key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(v_key, caller_id);
    auto val = param_server_.getParam(key);
    setResponse(result, 1, "Parameter [" + key + "]", val);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
  catch (const std::out_of_range&)
  {
    setResponse(result, -1, "Parameter [" + key + "] is not set", XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleSearchParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue("")))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string key = params[1];

  try
  {
    nonEmptyStr("key")(key, caller_id);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(""));
    return;
  }

  std::string search_key = param_server_.searchParam(caller_id, key);
  if (!search_key.empty())
  {
    setResponse(result, 1, "Found [" + search_key + "]", XmlRpc::XmlRpcValue(search_key));
  }
  else
  {
    setResponse(result, -1, "Cannot find parameter [" + key + "] in an upwards search",
                 XmlRpc::XmlRpcValue(""));
  }
}

void ROSMasterHandler::handleSubscribeParam(XmlRpc::XmlRpcValue& params,
                                              XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string caller_api = params[1];
  std::string key = params[2];

  try
  {
    caller_api = isApi("caller_api")(caller_api, caller_id);
    key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(key, caller_id);

    XmlRpc::XmlRpcValue val;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      val = param_server_.subscribeParam(key, caller_id, caller_api);
    }
    mloginfo("+CACHEDPARAM [" + key + "] by " + caller_id);
    setResponse(result, 1, "Subscribed to parameter [" + key + "]", val);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleUnsubscribeParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string caller_api = params[1];
  std::string key = params[2];

  try
  {
    caller_api = isApi("caller_api")(caller_api, caller_id);
    key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(key, caller_id);

    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      param_server_.unsubscribeParam(key, caller_id, caller_api);
    }
    mloginfo("-CACHEDPARAM [" + key + "] by " + caller_id);
    setResponse(result, 1, "Unsubscribe to parameter [" + key + "]", XmlRpc::XmlRpcValue(1));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleHasParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue(false)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string key = params[1];

  try
  {
    key = nonEmptyStr("key")(key, caller_id);
    key = names::resolveName(key, caller_id);
    bool has = param_server_.hasParam(key);
    setResponse(result, 1, key, XmlRpc::XmlRpcValue(has));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(false));
  }
}

void ROSMasterHandler::handleGetParamNames(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 1, XmlRpc::XmlRpcValue()))
  {
    return;
  }

  auto names_list = param_server_.getParamNames();
  XmlRpc::XmlRpcValue names_val;
  names_val.setSize(static_cast<int>(names_list.size()));
  for (int i = 0; i < static_cast<int>(names_list.size()); ++i)
  {
    names_val[i] = names_list[i];
  }
  setResponse(result, 1, "Parameter names", names_val);
}

// --- Service methods ---

void ROSMasterHandler::handleRegisterService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 4, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string service = params[1];
  std::string service_api = params[2];
  std::string caller_api = params[3];

  try
  {
    service = isService("service")(service, caller_id);
    service_api = isApi("service_api")(service_api, caller_id);
    caller_api = isApi("caller_api")(caller_api, caller_id);

    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      reg_manager_.registerService(service, caller_id, caller_api, service_api);
    }
    mloginfo("+SERVICE [" + service + "] " + caller_id + " " + caller_api);
    setResponse(result, 1,
                 "Registered [" + caller_id + "] as provider of [" + service + "]",
                 XmlRpc::XmlRpcValue(1));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleLookupService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue("")))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string service = params[1];

  try
  {
    service = isService("service")(service, caller_id);

    std::string service_url;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      service_url = reg_manager_.getServices().getServiceApi(service);
    }
    if (!service_url.empty())
    {
      setResponse(result, 1, "rosrpc URI: [" + service_url + "]",
                   XmlRpc::XmlRpcValue(service_url));
    }
    else
    {
      setResponse(result, -1, "no provider", XmlRpc::XmlRpcValue(""));
    }
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(""));
  }
}

void ROSMasterHandler::handleUnregisterService(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string service = params[1];
  std::string service_api = params[2];

  try
  {
    service = isService("service")(service, caller_id);
    service_api = isApi("service_api")(service_api, caller_id);

    std::tuple<int, std::string, int> retval;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      retval = reg_manager_.unregisterService(service, caller_id, service_api);
    }
    mloginfo("-SERVICE [" + service + "] " + caller_id + " " + service_api);
    setResponse(result, std::get<0>(retval), std::get<1>(retval),
                 XmlRpc::XmlRpcValue(std::get<2>(retval)));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

// --- Pub/Sub methods ---

void ROSMasterHandler::handleRegisterSubscriber(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  XmlRpc::XmlRpcValue empty_array;
  empty_array.setSize(0);
  if (!validateCallerId(params, result, 4, empty_array))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string topic = params[1];
  std::string topic_type = params[2];
  std::string caller_api = params[3];

  try
  {
    topic = isTopic("topic")(topic, caller_id);
    topic_type = validTypeName("topic_type")(topic_type, caller_id);
    caller_api = isApi("caller_api")(caller_api, caller_id);

    std::vector<std::string> pub_uris;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      reg_manager_.registerSubscriber(topic, caller_id, caller_api);
      if (topics_types_.find(topic) == topics_types_.end() && topic_type != names::ANYTYPE)
      {
        topics_types_[topic] = topic_type;
      }
      mloginfo("+SUB [" + topic + "] " + caller_id + " " + caller_api);
      pub_uris = reg_manager_.getPublishers().getApis(topic);
    }

    XmlRpc::XmlRpcValue pub_uris_val;
    pub_uris_val.setSize(static_cast<int>(pub_uris.size()));
    for (int i = 0; i < static_cast<int>(pub_uris.size()); ++i)
    {
      pub_uris_val[i] = pub_uris[i];
    }
    setResponse(result, 1, "Subscribed to [" + topic + "]", pub_uris_val);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), empty_array);
  }
}

void ROSMasterHandler::handleUnregisterSubscriber(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string topic = params[1];
  std::string caller_api = params[2];

  try
  {
    topic = isTopic("topic")(topic, caller_id);
    caller_api = isApi("caller_api")(caller_api, caller_id);

    std::tuple<int, std::string, int> retval;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      retval = reg_manager_.unregisterSubscriber(topic, caller_id, caller_api);
    }
    mloginfo("-SUB [" + topic + "] " + caller_id + " " + caller_api);
    setResponse(result, std::get<0>(retval), std::get<1>(retval),
                 XmlRpc::XmlRpcValue(std::get<2>(retval)));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

void ROSMasterHandler::handleRegisterPublisher(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  XmlRpc::XmlRpcValue empty_array;
  empty_array.setSize(0);
  if (!validateCallerId(params, result, 4, empty_array))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string topic = params[1];
  std::string topic_type = params[2];
  std::string caller_api = params[3];

  try
  {
    topic = isTopic("topic")(topic, caller_id);
    topic_type = validTypeName("topic_type")(topic_type, caller_id);
    caller_api = isApi("caller_api")(caller_api, caller_id);

    std::vector<std::string> sub_uris;
    std::vector<std::string> pub_uris;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      reg_manager_.registerPublisher(topic, caller_id, caller_api);
      if (topic_type != names::ANYTYPE || topics_types_.find(topic) == topics_types_.end())
      {
        topics_types_[topic] = topic_type;
      }
      pub_uris = reg_manager_.getPublishers().getApis(topic);
      sub_uris = reg_manager_.getSubscribers().getApis(topic);
      mloginfo("+PUB [" + topic + "] " + caller_id + " " + caller_api);
    }
    notifyTopicSubscribers(topic, pub_uris, sub_uris);

    XmlRpc::XmlRpcValue sub_uris_val;
    sub_uris_val.setSize(static_cast<int>(sub_uris.size()));
    for (int i = 0; i < static_cast<int>(sub_uris.size()); ++i)
    {
      sub_uris_val[i] = sub_uris[i];
    }
    setResponse(result, 1,
                 "Registered [" + caller_id + "] as publisher of [" + topic + "]",
                 sub_uris_val);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), empty_array);
  }
}

void ROSMasterHandler::handleUnregisterPublisher(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 3, XmlRpc::XmlRpcValue(0)))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string topic = params[1];
  std::string caller_api = params[2];

  try
  {
    topic = isTopic("topic")(topic, caller_id);
    caller_api = isApi("caller_api")(caller_api, caller_id);

    std::tuple<int, std::string, int> retval;
    std::vector<std::string> pub_uris_notify;
    std::vector<std::string> sub_uris_notify;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      retval = reg_manager_.unregisterPublisher(topic, caller_id, caller_api);
      if (std::get<2>(retval) == 1)
      {
        pub_uris_notify = reg_manager_.getPublishers().getApis(topic);
        sub_uris_notify = reg_manager_.getSubscribers().getApis(topic);
      }
    }
    if (std::get<2>(retval) == 1)
    {
      notifyTopicSubscribers(topic, pub_uris_notify, sub_uris_notify);
    }
    mloginfo("-PUB [" + topic + "] " + caller_id + " " + caller_api);
    setResponse(result, std::get<0>(retval), std::get<1>(retval),
                 XmlRpc::XmlRpcValue(std::get<2>(retval)));
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(0));
  }
}

// --- Graph State methods ---

void ROSMasterHandler::handleLookupNode(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  if (!validateCallerId(params, result, 2, XmlRpc::XmlRpcValue("")))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string node_name = params[1];

  try
  {
    node_name = validName("node")(node_name, caller_id);

    std::lock_guard<std::mutex> lock(ps_lock_);
    auto* node = reg_manager_.getNode(node_name);
    if (node != nullptr)
    {
      setResponse(result, 1, "node api", XmlRpc::XmlRpcValue(node->api));
    }
    else
    {
      setResponse(result, -1, "unknown node [" + node_name + "]", XmlRpc::XmlRpcValue(""));
    }
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), XmlRpc::XmlRpcValue(""));
  }
}

void ROSMasterHandler::handleGetPublishedTopics(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  XmlRpc::XmlRpcValue empty_array;
  empty_array.setSize(0);
  if (!validateCallerId(params, result, 2, empty_array))
  {
    return;
  }
  std::string caller_id = params[0];
  std::string subgraph = params[1];

  try
  {
    subgraph = emptyOrValidName("subgraph")(subgraph, caller_id);

    // Force subgraph to be a namespace with trailing slash
    if (!subgraph.empty() && subgraph.back() != '/')
    {
      subgraph += "/";
    }

    XmlRpc::XmlRpcValue topics_val;
    {
      std::lock_guard<std::mutex> lock(ps_lock_);
      auto keys = reg_manager_.getPublishers().getKeys();
      int idx = 0;
      for (const auto& t : keys)
      {
        if (t.substr(0, subgraph.size()) == subgraph)
        {
          auto type_it = topics_types_.find(t);
          if (type_it != topics_types_.end())
          {
            XmlRpc::XmlRpcValue pair;
            pair.setSize(2);
            pair[0] = t;
            pair[1] = type_it->second;
            topics_val[idx++] = pair;
          }
        }
      }
      if (idx == 0)
      {
        topics_val.setSize(0);
      }
    }
    setResponse(result, 1, "current topics", topics_val);
  }
  catch (const ParameterInvalid& e)
  {
    setResponse(result, -1, std::string(e.what()), empty_array);
  }
}

void ROSMasterHandler::handleGetTopicTypes(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  XmlRpc::XmlRpcValue empty_array;
  empty_array.setSize(0);
  if (!validateCallerId(params, result, 1, empty_array))
  {
    return;
  }

  XmlRpc::XmlRpcValue types_val;
  {
    std::lock_guard<std::mutex> lock(ps_lock_);
    int idx = 0;
    for (const auto& kv : topics_types_)
    {
      XmlRpc::XmlRpcValue pair;
      pair.setSize(2);
      pair[0] = kv.first;
      pair[1] = kv.second;
      types_val[idx++] = pair;
    }
    if (idx == 0)
    {
      types_val.setSize(0);
    }
  }
  setResponse(result, 1, "current system state", types_val);
}

void ROSMasterHandler::handleGetSystemState(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  XmlRpc::XmlRpcValue empty_state;
  empty_state.setSize(3);
  empty_state[0].setSize(0);
  empty_state[1].setSize(0);
  empty_state[2].setSize(0);

  if (!validateCallerId(params, result, 1, empty_state))
  {
    return;
  }

  auto convertState = [](const std::vector<std::pair<std::string, std::vector<std::string>>>& state)
      -> XmlRpc::XmlRpcValue {
    XmlRpc::XmlRpcValue val;
    if (state.empty())
    {
      val.setSize(0);
      return val;
    }
    val.setSize(static_cast<int>(state.size()));
    for (int i = 0; i < static_cast<int>(state.size()); ++i)
    {
      XmlRpc::XmlRpcValue entry;
      entry.setSize(2);
      entry[0] = state[i].first;
      XmlRpc::XmlRpcValue ids;
      ids.setSize(static_cast<int>(state[i].second.size()));
      for (int j = 0; j < static_cast<int>(state[i].second.size()); ++j)
      {
        ids[j] = state[i].second[j];
      }
      entry[1] = ids;
      val[i] = entry;
    }
    return val;
  };

  XmlRpc::XmlRpcValue sys_state;
  {
    std::lock_guard<std::mutex> lock(ps_lock_);
    sys_state.setSize(3);
    sys_state[0] = convertState(reg_manager_.getPublishers().getState());
    sys_state[1] = convertState(reg_manager_.getSubscribers().getState());
    sys_state[2] = convertState(reg_manager_.getServices().getState());
  }
  setResponse(result, 1, "current system state", sys_state);
}

}  // namespace rosmaster
