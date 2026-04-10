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

#include "rosmaster/registrations.h"
#include "rosmaster/thread_pool.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "xmlrpcpp/XmlRpcClient.h"
#include "xmlrpcpp/XmlRpcValue.h"

namespace rosmaster
{

// --- NodeRef ---

void NodeRef::clear()
{
  param_subscriptions.clear();
  topic_subscriptions.clear();
  topic_publications.clear();
  services.clear();
}

bool NodeRef::isEmpty() const
{
  return param_subscriptions.empty() && topic_subscriptions.empty() && topic_publications.empty() &&
         services.empty();
}

void NodeRef::add(int type, const std::string& key)
{
  switch (type)
  {
    case Registrations::TOPIC_SUBSCRIPTIONS:
      topic_subscriptions.insert(key);
      break;
    case Registrations::TOPIC_PUBLICATIONS:
      topic_publications.insert(key);
      break;
    case Registrations::SERVICE:
      services.insert(key);
      break;
    case Registrations::PARAM_SUBSCRIPTIONS:
      param_subscriptions.insert(key);
      break;
    default:
      throw std::runtime_error("NodeRef::add: invalid registration type");
  }
}

void NodeRef::remove(int type, const std::string& key)
{
  switch (type)
  {
    case Registrations::TOPIC_SUBSCRIPTIONS:
      topic_subscriptions.erase(key);
      break;
    case Registrations::TOPIC_PUBLICATIONS:
      topic_publications.erase(key);
      break;
    case Registrations::SERVICE:
      services.erase(key);
      break;
    case Registrations::PARAM_SUBSCRIPTIONS:
      param_subscriptions.erase(key);
      break;
    default:
      throw std::runtime_error("NodeRef::remove: invalid registration type");
  }
}

// --- shutdownNodeTask: notify a node to shut down ---

static void shutdownNodeTask(const std::string& api, const std::string& caller_id,
                             const std::string& reason)
{
  try
  {
    // Parse URI: http://host:port/
    std::string host;
    int port = 0;
    std::string uri_path = "/";
    if (api.substr(0, 7) == "http://")
    {
      auto rest = api.substr(7);
      auto slash_pos = rest.find('/');
      std::string host_port = (slash_pos != std::string::npos) ? rest.substr(0, slash_pos) : rest;
      if (slash_pos != std::string::npos)
      {
        uri_path = rest.substr(slash_pos);
      }
      auto colon_pos = host_port.rfind(':');
      if (colon_pos != std::string::npos)
      {
        host = host_port.substr(0, colon_pos);
        port = std::stoi(host_port.substr(colon_pos + 1));
      }
    }
    if (host.empty() || port == 0)
    {
      return;
    }

    XmlRpc::XmlRpcClient client(host.c_str(), port, uri_path.c_str());
    XmlRpc::XmlRpcValue params;
    params[0] = "/master";
    params[1] = "[" + caller_id + "] Reason: " + reason;
    XmlRpc::XmlRpcValue result;
    client.execute("shutdown", params, result);
  }
  catch (...)
  {
    // Expected in many common cases
  }
}

// --- Registrations ---

Registrations::Registrations(Type type) : type_(type)
{
}

std::vector<std::string> Registrations::getApis(const std::string& key) const
{
  std::vector<std::string> apis;
  auto it = map_.find(key);
  if (it != map_.end())
  {
    for (const auto& provider : it->second)
    {
      apis.push_back(provider.second);
    }
  }
  return apis;
}

std::string Registrations::getServiceApi(const std::string& service) const
{
  auto it = service_api_map_.find(service);
  if (it != service_api_map_.end())
  {
    return it->second.second;
  }
  return {};
}

bool Registrations::hasKey(const std::string& key) const
{
  return map_.find(key) != map_.end();
}

Registrations::ProviderList Registrations::getProviders(const std::string& key) const
{
  auto it = map_.find(key);
  if (it != map_.end())
  {
    return it->second;
  }
  return {};
}

std::vector<std::pair<std::string, std::vector<std::string>>> Registrations::getState() const
{
  std::vector<std::pair<std::string, std::vector<std::string>>> retval;
  for (const auto& kv : map_)
  {
    std::vector<std::string> ids;
    for (const auto& provider : kv.second)
    {
      ids.push_back(provider.first);
    }
    retval.emplace_back(kv.first, std::move(ids));
  }
  return retval;
}

std::vector<std::string> Registrations::getKeys() const
{
  std::vector<std::string> keys;
  for (const auto& kv : map_)
  {
    keys.push_back(kv.first);
  }
  return keys;
}

void Registrations::registerProvider(const std::string& key, const std::string& caller_id,
                                     const std::string& caller_api, const std::string& service_api)
{
  auto it = map_.find(key);
  if (it != map_.end() && service_api.empty())
  {
    auto& providers = it->second;
    Provider entry{caller_id, caller_api};
    // ProviderLists are typically very small (1-3 entries per topic)
    // so linear search is fine; avoiding overhead of a set
    bool found = false;
    for (const auto& p : providers)
    {
      if (p == entry) { found = true; break; }
    }
    if (!found)
    {
      providers.push_back(std::move(entry));
    }
  }
  else
  {
    map_[key] = {{caller_id, caller_api}};
  }

  if (!service_api.empty())
  {
    service_api_map_[key] = {caller_id, service_api};
  }
  else if (type_ == SERVICE)
  {
    throw std::runtime_error("service_api must be specified for SERVICE registrations");
  }
}

void Registrations::unregisterAll(const std::string& caller_id)
{
  std::vector<std::string> dead_keys;
  for (auto& kv : map_)
  {
    auto& providers = kv.second;
    providers.erase(std::remove_if(providers.begin(), providers.end(),
                                   [&](const Provider& p) { return p.first == caller_id; }),
                    providers.end());
    if (providers.empty())
    {
      dead_keys.push_back(kv.first);
    }
  }
  for (const auto& k : dead_keys)
  {
    map_.erase(k);
  }

  if (type_ == SERVICE)
  {
    dead_keys.clear();
    for (const auto& kv : service_api_map_)
    {
      if (kv.second.first == caller_id)
      {
        dead_keys.push_back(kv.first);
      }
    }
    for (const auto& k : dead_keys)
    {
      service_api_map_.erase(k);
    }
  }
}

std::tuple<int, std::string, int> Registrations::unregister(const std::string& key,
                                                             const std::string& caller_id,
                                                             const std::string& caller_api,
                                                             const std::string& service_api)
{
  if (!service_api.empty())
  {
    // Service unregistration: validate against service_api
    auto it = service_api_map_.find(key);
    if (it == service_api_map_.end())
    {
      return {1, "[" + caller_id + "] is not a provider of [" + key + "]", 0};
    }
    if (it->second != std::make_pair(caller_id, service_api))
    {
      return {1, "[" + service_api + "] is no longer the current service api handle for [" + key + "]", 0};
    }
    service_api_map_.erase(key);
    map_.erase(key);
    return {1, "Unregistered [" + caller_id + "] as provider of [" + key + "]", 1};
  }
  else if (type_ == SERVICE)
  {
    throw std::runtime_error("service_api must be specified for SERVICE unregistration");
  }

  auto map_it = map_.find(key);
  if (map_it != map_.end())
  {
    auto& providers = map_it->second;
    Provider entry{caller_id, caller_api};
    auto it = std::find(providers.begin(), providers.end(), entry);
    if (it != providers.end())
    {
      providers.erase(it);
      if (providers.empty())
      {
        map_.erase(map_it);
      }
      return {1, "Unregistered [" + caller_id + "] as provider of [" + key + "]", 1};
    }
  }
  return {1, "[" + caller_id + "] is not a known provider of [" + key + "]", 0};
}

// --- RegistrationManager ---

RegistrationManager::RegistrationManager(MarkedThreadPool* thread_pool)
  : publishers(Registrations::TOPIC_PUBLICATIONS)
  , subscribers(Registrations::TOPIC_SUBSCRIPTIONS)
  , services(Registrations::SERVICE)
  , param_subscribers(Registrations::PARAM_SUBSCRIPTIONS)
  , thread_pool_(thread_pool)
{
}

void RegistrationManager::registerService(const std::string& service, const std::string& caller_id,
                                           const std::string& caller_api,
                                           const std::string& service_api)
{
  doRegister(services, service, caller_id, caller_api, service_api);
}

void RegistrationManager::registerPublisher(const std::string& topic, const std::string& caller_id,
                                             const std::string& caller_api)
{
  doRegister(publishers, topic, caller_id, caller_api);
}

void RegistrationManager::registerSubscriber(const std::string& topic, const std::string& caller_id,
                                              const std::string& caller_api)
{
  doRegister(subscribers, topic, caller_id, caller_api);
}

void RegistrationManager::registerParamSubscriber(const std::string& param,
                                                   const std::string& caller_id,
                                                   const std::string& caller_api)
{
  doRegister(param_subscribers, param, caller_id, caller_api);
}

std::tuple<int, std::string, int>
RegistrationManager::unregisterService(const std::string& service, const std::string& caller_id,
                                        const std::string& service_api)
{
  return doUnregister(services, service, caller_id, "", service_api);
}

std::tuple<int, std::string, int>
RegistrationManager::unregisterSubscriber(const std::string& topic, const std::string& caller_id,
                                           const std::string& caller_api)
{
  return doUnregister(subscribers, topic, caller_id, caller_api);
}

std::tuple<int, std::string, int>
RegistrationManager::unregisterPublisher(const std::string& topic, const std::string& caller_id,
                                          const std::string& caller_api)
{
  return doUnregister(publishers, topic, caller_id, caller_api);
}

std::tuple<int, std::string, int>
RegistrationManager::unregisterParamSubscriber(const std::string& param,
                                                const std::string& caller_id,
                                                const std::string& caller_api)
{
  return doUnregister(param_subscribers, param, caller_id, caller_api);
}

NodeRef* RegistrationManager::getNode(const std::string& caller_id)
{
  auto it = nodes.find(caller_id);
  if (it != nodes.end())
  {
    return &it->second;
  }
  return nullptr;
}

std::vector<NodeRef*> RegistrationManager::reverseLookup(const std::string& caller_api)
{
  std::vector<NodeRef*> matches;
  auto it = api_to_nodes_.find(caller_api);
  if (it != api_to_nodes_.end())
  {
    for (const auto& node_id : it->second)
    {
      auto nit = nodes.find(node_id);
      if (nit != nodes.end())
      {
        matches.push_back(&nit->second);
      }
    }
  }
  return matches;
}

std::pair<NodeRef*, bool> RegistrationManager::registerNodeApi(const std::string& caller_id,
                                                                const std::string& caller_api)
{
  auto it = nodes.find(caller_id);
  if (it != nodes.end())
  {
    if (it->second.api == caller_api)
    {
      return {&it->second, false};
    }
    // Node re-registered with different API - update reverse index and shut down old node
    std::string bumped_api = it->second.api;
    api_to_nodes_[bumped_api].erase(caller_id);
    if (api_to_nodes_[bumped_api].empty())
    {
      api_to_nodes_.erase(bumped_api);
    }
    if (thread_pool_)
    {
      std::string cid = caller_id;
      thread_pool_->queueTask(bumped_api, [bumped_api, cid]() {
        shutdownNodeTask(bumped_api, cid, "new node registered with same name");
      });
    }
  }
  nodes[caller_id] = NodeRef(caller_id, caller_api);
  api_to_nodes_[caller_api].insert(caller_id);
  return {&nodes[caller_id], it != nodes.end()};
}

void RegistrationManager::doRegister(Registrations& r, const std::string& key,
                                      const std::string& caller_id, const std::string& caller_api,
                                      const std::string& service_api)
{
  auto [node_ref, changed] = registerNodeApi(caller_id, caller_api);
  node_ref->add(r.getType(), key);
  if (changed)
  {
    publishers.unregisterAll(caller_id);
    subscribers.unregisterAll(caller_id);
    services.unregisterAll(caller_id);
    param_subscribers.unregisterAll(caller_id);
  }
  r.registerProvider(key, caller_id, caller_api, service_api);
}

std::tuple<int, std::string, int>
RegistrationManager::doUnregister(Registrations& r, const std::string& key,
                                   const std::string& caller_id, const std::string& caller_api,
                                   const std::string& service_api)
{
  auto it = nodes.find(caller_id);
  if (it != nodes.end())
  {
    auto retval = r.unregister(key, caller_id, caller_api, service_api);
    if (std::get<2>(retval) == 1)
    {
      it->second.remove(r.getType(), key);
    }
    if (it->second.isEmpty())
    {
      std::string api = it->second.api;
      api_to_nodes_[api].erase(caller_id);
      if (api_to_nodes_[api].empty())
      {
        api_to_nodes_.erase(api);
      }
      nodes.erase(it);
    }
    return retval;
  }
  return {1, "[" + caller_id + "] is not a registered node", 0};
}

}  // namespace rosmaster
