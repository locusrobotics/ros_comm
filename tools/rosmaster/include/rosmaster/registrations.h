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

#ifndef ROSMASTER_REGISTRATIONS_H
#define ROSMASTER_REGISTRATIONS_H

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rosmaster
{

class MarkedThreadPool;

/// Container for node registration information
struct NodeRef
{
  std::string id;
  std::string api;
  std::unordered_set<std::string> param_subscriptions;
  std::unordered_set<std::string> topic_subscriptions;
  std::unordered_set<std::string> topic_publications;
  std::unordered_set<std::string> services;

  NodeRef() = default;
  NodeRef(const std::string& id, const std::string& api) : id(id), api(api) {}

  void clear();
  bool isEmpty() const;
  void add(int type, const std::string& key);
  void remove(int type, const std::string& key);
};

/// A multimap for storing registrations (publishers, subscribers, services, param subscribers)
class Registrations
{
public:
  enum Type
  {
    TOPIC_SUBSCRIPTIONS = 1,
    TOPIC_PUBLICATIONS = 2,
    SERVICE = 3,
    PARAM_SUBSCRIPTIONS = 4
  };

  using Provider = std::pair<std::string, std::string>;  // (caller_id, caller_api)
  using ProviderList = std::vector<Provider>;

  explicit Registrations(Type type);

  bool empty() const { return map_.empty(); }

  /// Get all API URIs for a key (not valid for SERVICE type)
  std::vector<std::string> getApis(const std::string& key) const;

  /// Get the service API for a service key
  std::string getServiceApi(const std::string& service) const;

  bool hasKey(const std::string& key) const;

  /// Get (caller_id, caller_api) pairs for a key
  ProviderList getProviders(const std::string& key) const;

  /// Get state in getSystemState()-friendly format
  std::vector<std::pair<std::string, std::vector<std::string>>> getState() const;

  /// Get all keys
  std::vector<std::string> getKeys() const;

  void registerProvider(const std::string& key, const std::string& caller_id,
                        const std::string& caller_api, const std::string& service_api = "");

  void unregisterAll(const std::string& caller_id);

  /// Returns (code, msg, num_unregistered)
  std::tuple<int, std::string, int> unregister(const std::string& key, const std::string& caller_id,
                                                const std::string& caller_api,
                                                const std::string& service_api = "");

  Type getType() const { return type_; }

private:
  Type type_;
  std::unordered_map<std::string, ProviderList> map_;
  // service_api_map: { key: (caller_id, service_api) } - only for SERVICE type
  std::unordered_map<std::string, std::pair<std::string, std::string>> service_api_map_;
};

/// Central registration tracker for the master
class RegistrationManager
{
public:
  explicit RegistrationManager(MarkedThreadPool* thread_pool);

  void registerService(const std::string& service, const std::string& caller_id,
                       const std::string& caller_api, const std::string& service_api);

  void registerPublisher(const std::string& topic, const std::string& caller_id,
                         const std::string& caller_api);

  void registerSubscriber(const std::string& topic, const std::string& caller_id,
                          const std::string& caller_api);

  void registerParamSubscriber(const std::string& param, const std::string& caller_id,
                               const std::string& caller_api);

  std::tuple<int, std::string, int> unregisterService(const std::string& service,
                                                       const std::string& caller_id,
                                                       const std::string& service_api);

  std::tuple<int, std::string, int> unregisterSubscriber(const std::string& topic,
                                                          const std::string& caller_id,
                                                          const std::string& caller_api);

  std::tuple<int, std::string, int> unregisterPublisher(const std::string& topic,
                                                         const std::string& caller_id,
                                                         const std::string& caller_api);

  std::tuple<int, std::string, int> unregisterParamSubscriber(const std::string& param,
                                                               const std::string& caller_id,
                                                               const std::string& caller_api);

  NodeRef* getNode(const std::string& caller_id);

  /// Reverse lookup: find nodes by their API URI
  std::vector<NodeRef*> reverseLookup(const std::string& caller_api);

  Registrations publishers;
  Registrations subscribers;
  Registrations services;
  Registrations param_subscribers;
  std::unordered_map<std::string, NodeRef> nodes;

private:
  /// Register or update a node's API. Returns (node_ref, api_changed)
  std::pair<NodeRef*, bool> registerNodeApi(const std::string& caller_id, const std::string& caller_api);

  void doRegister(Registrations& r, const std::string& key, const std::string& caller_id,
                  const std::string& caller_api, const std::string& service_api = "");

  std::tuple<int, std::string, int> doUnregister(Registrations& r, const std::string& key,
                                                  const std::string& caller_id,
                                                  const std::string& caller_api,
                                                  const std::string& service_api = "");

  MarkedThreadPool* thread_pool_;
  // Reverse index: api_uri -> set of node_ids (for O(1) reverseLookup)
  std::unordered_map<std::string, std::unordered_set<std::string>> api_to_nodes_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_REGISTRATIONS_H
