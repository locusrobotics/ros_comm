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

#ifndef ROSMASTER_PARAM_SERVER_H
#define ROSMASTER_PARAM_SERVER_H

#include "rosmaster/types.h"

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpcValue.h"

namespace rosmaster
{

class RegistrationManager;

/**
 * @brief Subscriber update payload: ([(caller_id, caller_api)...], param_key, param_value).
 */
struct ParamUpdate
{
  ProviderList subscribers;
  std::string key;
  XmlRpc::XmlRpcValue value;
};

using NotifyFunc = std::function<void(const std::vector<ParamUpdate>&)>;

/**
 * @brief Compute subscriber notifications for a parameter update.
 * @param subscribers Parameter subscribers registry.
 * @param param_key Updated parameter key.
 * @param param_value Updated parameter value.
 * @param caller_id_to_ignore Caller ID to exclude from notifications.
 * @return List of grouped parameter updates to dispatch.
 */
std::vector<ParamUpdate> computeParamUpdates(
  const class Registrations& subscribers,
  const std::string& param_key,
  const XmlRpc::XmlRpcValue& param_value,
  const std::string& caller_id_to_ignore = "");

/**
 * @brief Hierarchical parameter storage with subscriber notifications.
 */
class ParamDictionary
{
public:
  /**
   * @brief Construct a parameter dictionary.
   * @param reg_manager Registration manager used to track parameter subscribers.
   */
  explicit ParamDictionary(RegistrationManager* reg_manager);

  /**
   * @brief Get the list of all parameter names.
   * @return All parameter keys currently stored.
   */
  std::vector<std::string> getParamNames();

  /**
   * @brief Search for a parameter from a namespace upward.
   * @param ns Starting namespace.
   * @param key Parameter key to search for.
   * @return Resolved parameter key if found, otherwise an empty string.
   */
  std::string searchParam(const std::string& ns, const std::string& key);

  /**
   * @brief Get a parameter value.
   * @param key Parameter key.
   * @return Parameter value.
   * @throws std::out_of_range If the parameter is not found.
   */
  XmlRpc::XmlRpcValue getParam(const std::string& key);

  /**
   * @brief Set a parameter value with optional subscriber notification.
   * @param key Parameter key.
   * @param value Value to store.
   * @param notify_task Optional callback that dispatches batched notifications.
   * @param caller_id Caller ID associated with the update.
   */
  void setParam(
    const std::string& key,
    const XmlRpc::XmlRpcValue& value,
    NotifyFunc notify_task = nullptr,
    const std::string& caller_id = "");

  /**
   * @brief Delete a parameter.
   * @param key Parameter key.
   * @param notify_task Optional callback that dispatches batched notifications.
   * @throws std::out_of_range If the parameter is not found.
   */
  void deleteParam(const std::string& key, NotifyFunc notify_task = nullptr);

  /**
   * @brief Check whether a parameter exists.
   * @param key Parameter key.
   * @return True if the parameter exists.
   */
  bool hasParam(const std::string& key);

  /**
   * @brief Subscribe to parameter updates.
   * @param key Parameter key.
   * @param caller_id Subscriber caller ID.
   * @param caller_api Subscriber XML-RPC API URI.
   * @return Current parameter value.
   */
  XmlRpc::XmlRpcValue subscribeParam(
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Unsubscribe from parameter updates.
   * @param key Parameter key.
   * @param caller_id Subscriber caller ID.
   * @param caller_api Subscriber XML-RPC API URI.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unsubscribeParam(
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api);

private:
  /**
   * @brief Recursively collect parameter names from a dictionary node.
   * @param names Output vector receiving fully qualified parameter keys.
   * @param key Current parameter namespace/key prefix.
   * @param d Dictionary node being traversed.
   */
  void getParamNamesImpl(
    std::vector<std::string>& names,
    const std::string& key,
    const XmlRpc::XmlRpcValue& d);

  std::recursive_mutex lock_;
  XmlRpc::XmlRpcValue parameters_;
  RegistrationManager* reg_manager_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_PARAM_SERVER_H
