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

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpcValue.h"

namespace rosmaster
{

class RegistrationManager;

/// Subscriber update: ([(caller_id, caller_api)...], param_key, param_value)
struct ParamUpdate
{
  std::vector<std::pair<std::string, std::string>> subscribers;
  std::string key;
  XmlRpc::XmlRpcValue value;
};

using NotifyFunc = std::function<void(const std::vector<ParamUpdate>&)>;

/// Compute which subscribers should be notified based on a parameter update
std::vector<ParamUpdate> computeParamUpdates(const class Registrations& subscribers,
                                              const std::string& param_key,
                                              const XmlRpc::XmlRpcValue& param_value,
                                              const std::string& caller_id_to_ignore = "");

/// Hierarchical parameter storage with subscriber notifications
class ParamDictionary
{
public:
  explicit ParamDictionary(RegistrationManager* reg_manager);

  /// Get list of all parameter names
  std::vector<std::string> getParamNames();

  /// Search for parameter starting from namespace, going up
  std::string searchParam(const std::string& ns, const std::string& key);

  /// Get parameter value. Throws std::out_of_range if not found.
  XmlRpc::XmlRpcValue getParam(const std::string& key);

  /// Set parameter value with optional subscriber notification
  void setParam(const std::string& key, const XmlRpc::XmlRpcValue& value,
                NotifyFunc notify_task = nullptr, const std::string& caller_id = "");

  /// Delete parameter. Throws std::out_of_range if not found.
  void deleteParam(const std::string& key, NotifyFunc notify_task = nullptr);

  /// Check if parameter exists
  bool hasParam(const std::string& key);

  /// Subscribe to parameter updates. Returns current value.
  XmlRpc::XmlRpcValue subscribeParam(const std::string& key, const std::string& caller_id,
                                      const std::string& caller_api);

  /// Unsubscribe from parameter updates
  std::tuple<int, std::string, int> unsubscribeParam(const std::string& key,
                                                      const std::string& caller_id,
                                                      const std::string& caller_api);

private:
  void getParamNamesImpl(std::vector<std::string>& names, const std::string& key,
                          const XmlRpc::XmlRpcValue& d);

  std::recursive_mutex lock_;
  XmlRpc::XmlRpcValue parameters_;
  RegistrationManager* reg_manager_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_PARAM_SERVER_H
