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

#include "rosmaster/types.h"

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rosmaster
{

class MarkedThreadPool;

/**
 * @brief Container for node registration information.
 */
struct NodeRef
{
  std::string id;
  std::string api;
  std::unordered_set<std::string> param_subscriptions;
  std::unordered_set<std::string> topic_subscriptions;
  std::unordered_set<std::string> topic_publications;
  std::unordered_set<std::string> services;

  /**
   * @brief Construct an empty node reference.
   */
  NodeRef() = default;

  /**
   * @brief Construct a node reference with a caller ID and API URI.
   * @param id Node caller ID.
   * @param api Node XML-RPC API URI.
   */
  NodeRef(const std::string& id, const std::string& api) : id(id), api(api) {}

  /**
   * @brief Clear all registrations tracked by this node reference.
   */
  void clear();

  /**
   * @brief Check whether this node has no registrations in any category.
   * @return True if all registration sets are empty.
   */
  bool isEmpty() const;

  /**
   * @brief Add a key to the registration set for the given type.
   * @param type Registration type from Registrations::Type.
   * @param key Registration key to add.
   */
  void add(int type, const std::string& key);

  /**
   * @brief Remove a key from the registration set for the given type.
   * @param type Registration type from Registrations::Type.
   * @param key Registration key to remove.
   */
  void remove(int type, const std::string& key);
};

/**
 * @class A multimap for storing registrations (publishers, subscribers, services, param subscribers).
 */
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

  /**
   * @brief Construct a registrations table for a specific registration type.
   * @param type Registration type managed by this table.
   */
  explicit Registrations(Type type);

  /**
   * @brief Check whether there are no registered providers.
   * @return True if the registration map is empty.
   */
  bool empty() const { return map_.empty(); }

  /**
   * @brief Get all API URIs for a key (not valid for SERVICE type).
   * @param key Topic or parameter key to query.
   * @return List of provider API URIs for the key.
   */
  std::vector<std::string> getApis(const std::string& key) const;

  /**
   * @brief Get the service API for a service key.
   * @param service Service name.
   * @return Service API URI, or an empty string if not found.
   */
  std::string getServiceApi(const std::string& service) const;

  /**
   * @brief Check whether a key exists in this registration table.
   * @param key Registration key to check.
   * @return True if the key exists.
   */
  bool hasKey(const std::string& key) const;

  /**
   * @brief Get (caller_id, caller_api) pairs for a key.
   * @param key Registration key to query.
   * @return Provider list associated with the key.
   */
  ProviderList getProviders(const std::string& key) const;

  /**
   * @brief Get state in getSystemState()-friendly format.
   * @return List of key/provider-name pairs for system state reporting.
   */
  std::vector<std::pair<std::string, std::vector<std::string>>> getState() const;

  /**
   * @brief Get all keys.
   * @return All registered keys.
   */
  std::vector<std::string> getKeys() const;

  /**
   * @brief Register a provider for a key.
   * @param key Topic, service, or parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @param service_api Service API URI when registering services.
   */
  void registerProvider(
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& service_api = "");

  /**
   * @brief Remove all registrations belonging to a caller ID.
   * @param caller_id Node caller ID to purge.
   */
  void unregisterAll(const std::string& caller_id);

  /**
   * @brief Unregister a provider for a key.
   * @param key Topic, service, or parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @param service_api Service API URI for service unregistration.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unregister(
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& service_api = "");

  /**
   * @brief Get the registration type managed by this instance.
   * @return Registration type enum value.
   */
  Type getType() const { return type_; }

private:
  Type type_;

  // Map of key -> list of providers (caller_id, caller_api)
  std::unordered_map<std::string, ProviderList> map_;

  // Map of key->service_api for SERVICE type (for O(1) lookup during unregister)
  std::unordered_map<std::string, ServiceProvider> service_api_map_;
};

/**
 * @class Central registration tracker for the master.
 */
class RegistrationManager
{
public:
  /**
   * @brief Construct a registration manager.
   * @param thread_pool Thread pool used for asynchronous operations.
   */
  explicit RegistrationManager(MarkedThreadPool* thread_pool);

  /**
   * @brief Register a node as a parameter subscriber.
   * @param param Parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   */
  void registerParamSubscriber(
    const std::string& param,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Register a node as a topic publisher.
   * @param topic Topic name.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   */
  void registerPublisher(
    const std::string& topic,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Register a service provider.
   * @param service Service name.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @param service_api Service API URI.
   */
  void registerService(
    const std::string& service,
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& service_api);

  /**
   * @brief Register a node as a topic subscriber.
   * @param topic Topic name.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   */
  void registerSubscriber(
    const std::string& topic,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Unregister a parameter subscriber.
   * @param param Parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unregisterParamSubscriber(
    const std::string& param,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Unregister a topic publisher.
   * @param topic Topic name.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unregisterPublisher(
    const std::string& topic,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Unregister a service provider.
   * @param service Service name.
   * @param caller_id Node caller ID.
   * @param service_api Service API URI.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unregisterService(
    const std::string& service,
    const std::string& caller_id,
    const std::string& service_api);

  /**
   * @brief Unregister a topic subscriber.
   * @param topic Topic name.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> unregisterSubscriber(
    const std::string& topic,
    const std::string& caller_id,
    const std::string& caller_api);

  /**
   * @brief Find a tracked node by caller ID.
   * @param caller_id Node caller ID.
   * @return Pointer to the node reference, or null if unknown.
   */
  NodeRef* getNode(const std::string& caller_id);

  /**
   * @brief Access parameter subscriber registrations.
   * @return Mutable parameter subscriber registration table.
   */
  Registrations& getParamSubscribers();

  /**
   * @brief Access publisher registrations.
   * @return Mutable publisher registration table.
   */
  Registrations& getPublishers();

  /**
   * @brief Access service registrations.
   * @return Mutable service registration table.
   */
  Registrations& getServices();

  /**
   * @brief Access subscriber registrations.
   * @return Mutable subscriber registration table.
   */
  Registrations& getSubscribers();

  /**
   * @brief Reverse lookup: find nodes by their API URI.
   * @param caller_api Node XML-RPC API URI.
   * @return Node references that use the specified API URI.
   */
  std::vector<NodeRef*> reverseLookup(const std::string& caller_api);
  
private:
  /**
   * @brief Register or update a node's API.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @return Pair of node reference pointer and whether the API changed.
   */
  std::pair<NodeRef*, bool> registerNodeApi(const std::string& caller_id, const std::string& caller_api);

  /**
   * @brief Internal helper to add a registration and update node bookkeeping.
   * @param r Registration table to update.
   * @param key Topic, service, or parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @param service_api Service API URI for service registrations.
   */
  void doRegister(
    Registrations& r,
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& service_api = "");

  /**
   * @brief Internal helper to remove a registration and update node bookkeeping.
   * @param r Registration table to update.
   * @param key Topic, service, or parameter key.
   * @param caller_id Node caller ID.
   * @param caller_api Node XML-RPC API URI.
   * @param service_api Service API URI for service unregistrations.
   * @return Tuple of status code, status message, and unregistered count.
   */
  std::tuple<int, std::string, int> doUnregister(
    Registrations& r,
    const std::string& key,
    const std::string& caller_id,
    const std::string& caller_api,
    const std::string& service_api = "");

  Registrations publishers_;
  Registrations subscribers_;
  Registrations services_;
  Registrations param_subscribers_;

  std::unordered_map<std::string, NodeRef> nodes_;

  MarkedThreadPool* thread_pool_;

  // Reverse index: api_uri -> set of node_ids (for O(1) reverseLookup)
  std::unordered_map<std::string, std::unordered_set<std::string>> api_to_nodes_;
};

}  // namespace rosmaster

#endif  // ROSMASTER_REGISTRATIONS_H
