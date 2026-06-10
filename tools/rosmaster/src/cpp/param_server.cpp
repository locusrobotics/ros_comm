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

#include "rosmaster/param_server.h"
#include "rosmaster/names.h"
#include "rosmaster/registrations.h"
#include "rosmaster/utilities.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace rosmaster
{

namespace
{

/**
 * @brief Recursively remove internal TypeInvalid placeholder entries from XmlRpcValue structs.
 * @details Placeholder entries (TypeInvalid value) are used internally to force TypeStruct
 * promotion on empty namespaces. Only entries with TypeInvalid values are stripped, so user
 * parameters whose keys happen to match placeholder names are preserved.
 * structToXml() already skips TypeInvalid members, so placeholders won't appear in XML.
 * @param src XmlRpc value to sanitize.
 * @return XmlRpc value with internal placeholder entries removed.
 */
XmlRpc::XmlRpcValue stripPlaceholders(const XmlRpc::XmlRpcValue& src)
{
  if (src.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    return src;
  }
  XmlRpc::XmlRpcValue result;
  bool has_real_key = false;
  for (auto it = src.begin(); it != src.end(); ++it)
  {
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    {
      continue;
    }
    result[it->first] = stripPlaceholders(it->second);
    has_real_key = true;
  }
  if (!has_real_key)
  {
    // Force TypeStruct promotion. The resulting __placeholder__ entry has
    // TypeInvalid value, which structToXml() will skip (see XmlRpcValue.cpp).
    result["__placeholder__"];
  }
  return result;
}

/**
 * @brief Collect all parameter keys contained in a nested XmlRpc struct.
 * @param param_key Current parameter namespace prefix.
 * @param param_value Parameter value to traverse.
 * @param all_keys Output set of discovered keys.
 */
void computeAllKeys(
  const std::string& param_key,
  const XmlRpc::XmlRpcValue& param_value,
  std::unordered_set<std::string>& all_keys)
{
  if (param_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    return;
  }
  for (auto it = param_value.begin(); it != param_value.end(); ++it)
  {
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    {
      continue;
    }
    std::string new_k = names::nsJoin(param_key, it->first) + "/";
    all_keys.insert(new_k);
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      computeAllKeys(new_k, it->second, all_keys);
    }
  }
}

}  // anonymous namespace

std::vector<ParamUpdate> computeParamUpdates(
  const Registrations& subscribers,
  const std::string& param_key,
  const XmlRpc::XmlRpcValue& param_value,
  const std::string& caller_id_to_ignore)
{
  if (subscribers.empty())
  {
    return {};
  }

  std::string pk = param_key;
  if (pk != "/")
  {
    pk = names::canonicalizeName(pk) + "/";
  }

  // Compute all updated keys if value is a dict
  std::unordered_set<std::string> all_keys;
  bool has_all_keys = false;
  if (param_value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    computeAllKeys(pk, param_value, all_keys);
    has_all_keys = true;
  }

  std::vector<ParamUpdate> updates;

  for (const auto& sub_key : subscribers.getKeys())
  {
    std::string ns_key = sub_key;
    if (ns_key.empty() || ns_key.back() != '/')
    {
      ns_key += "/";
    }

    if (pk.substr(0, ns_key.size()) == ns_key)
    {
      // Subscriber's namespace is a prefix of the updated key
      auto node_apis = subscribers.getProviders(sub_key);
      if (!caller_id_to_ignore.empty())
      {
        node_apis.erase(
            std::remove_if(node_apis.begin(), node_apis.end(),
                           [&](const auto& p) { return p.first == caller_id_to_ignore; }),
            node_apis.end());
      }
      ParamUpdate update;
      update.subscribers = std::move(node_apis);
      update.key = pk;
      update.value = param_value;
      updates.push_back(std::move(update));
    }
    else if (has_all_keys && ns_key.substr(0, pk.size()) == pk)
    {
      // Check if subscription was deleted (key is within the param namespace but not in new value)
      if (all_keys.count(ns_key) == 0)
      {
        auto node_apis = subscribers.getProviders(sub_key);
        ParamUpdate update;
        update.subscribers = std::move(node_apis);
        update.key = sub_key;

        // Signal parameter deletion with an empty struct {}, matching Python rosmaster behaviour.
        // We promote to TypeStruct via a sentinel key set to TypeInvalid; structToXml skips
        // TypeInvalid members, so this serialises as <struct></struct> = {}.
        update.value["__empty__"] = XmlRpc::XmlRpcValue();
        updates.push_back(std::move(update));
      }
    }
  }

  // Add updates for exact matches within tree
  if (has_all_keys)
  {
    for (const auto& key : all_keys)
    {
      if (subscribers.hasKey(key))
      {
        // Compute actual update value by navigating into param_value
        std::string sub_key = key.substr(pk.size());
        auto ns_parts = splitString(sub_key, '/');
        XmlRpc::XmlRpcValue val = param_value;
        for (const auto& ns : ns_parts)
        {
          if (val.getType() == XmlRpc::XmlRpcValue::TypeStruct && val.hasMember(ns))
          {
            val = val[ns];
          }
          else
          {
            break;
          }
        }
        ParamUpdate update;
        update.subscribers = subscribers.getProviders(key);
        update.key = key;
        update.value = val;
        updates.push_back(std::move(update));
      }
    }
  }

  return updates;
}

ParamDictionary::ParamDictionary(RegistrationManager* reg_manager) : reg_manager_(reg_manager)
{
  // Initialize parameters_ as a struct by accessing a key.
  // XmlRpcValue operator[] with string key auto-promotes to TypeStruct.
  parameters_["__placeholder__"];
}

void ParamDictionary::getParamNamesImpl(std::vector<std::string>& names, const std::string& key,
                                         const XmlRpc::XmlRpcValue& d)
{
  if (d.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    return;
  }
  for (auto it = d.begin(); it != d.end(); ++it)
  {
    // Skip internal TypeInvalid placeholder entries
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    {
      continue;
    }
    std::string full_key = names::nsJoin(key, it->first);
    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      getParamNamesImpl(names, full_key, it->second);
    }
    else
    {
      names.push_back(full_key);
    }
  }
}

std::vector<std::string> ParamDictionary::getParamNames()
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  std::vector<std::string> param_names;
  getParamNamesImpl(param_names, "/", parameters_);
  return param_names;
}

std::string ParamDictionary::searchParam(const std::string& ns, const std::string& key)
{
  if (key.empty() || names::isPrivate(key))
  {
    return {};
  }
  if (!names::isGlobal(ns))
  {
    return {};
  }
  if (names::isGlobal(key))
  {
    if (hasParam(key))
    {
      return key;
    }
    return {};
  }

  // Get the first namespace component of the key
  auto parts = splitString(key, '/');
  if (parts.empty())
  {
    return {};
  }
  std::string key_ns = parts[0];

  // Test initial namespace
  std::string search_key = names::nsJoin(ns, key_ns);
  if (hasParam(search_key))
  {
    return names::nsJoin(ns, key);
  }

  // Walk up the namespace tree
  auto ns_parts = splitString(ns, '/');
  for (size_t i = 1; i <= ns_parts.size(); ++i)
  {
    std::string prefix = "/";
    for (size_t j = 0; j < ns_parts.size() - i; ++j)
    {
      prefix += ns_parts[j] + "/";
    }
    search_key = prefix + key_ns;
    if (hasParam(search_key))
    {
      return prefix + key;
    }
  }
  return {};
}

XmlRpc::XmlRpcValue ParamDictionary::getParam(const std::string& key)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);

  if (key == names::GLOBALNS)
  {
    return stripPlaceholders(parameters_);
  }

  auto ns_parts = splitString(key, '/');

  // Use a pointer to walk the tree to avoid self-assignment issues
  const XmlRpc::XmlRpcValue* d = &parameters_;
  for (const auto& ns : ns_parts)
  {
    if (d->getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::out_of_range("Parameter [" + key + "] is not set");
    }
    if (!d->hasMember(ns))
    {
      throw std::out_of_range("Parameter [" + key + "] is not set");
    }
    d = &((*d)[ns]);
  }
  return stripPlaceholders(*d);
}

void ParamDictionary::setParam(
  const std::string& key,
  const XmlRpc::XmlRpcValue& value,
  NotifyFunc notify_task,
  const std::string& caller_id)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);

  if (key == names::GLOBALNS)
  {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::runtime_error("cannot set root of parameter tree to non-dictionary");
    }
    parameters_ = value;
  }
  else
  {
    auto ns_parts = splitString(key, '/');
    if (ns_parts.empty())
    {
      return;
    }
    std::string value_key = ns_parts.back();
    ns_parts.pop_back();

    // Ensure parameters_ is a struct
    if (!parameters_.valid() || parameters_.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      // Force struct type by accessing a key. We use the actual first key we need.
      parameters_[ns_parts.empty() ? value_key : ns_parts[0]];
    }

    // Navigate/create the namespace tree
    XmlRpc::XmlRpcValue* d = &parameters_;
    for (const auto& ns : ns_parts)
    {
      if (!d->hasMember(ns) || (*d)[ns].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        // Accessing a non-existent key on a struct auto-creates it as invalid.
        // We need to explicitly make it a struct by accessing a sub-key.
        (*d)[ns]["__placeholder__"];
      }
      d = &((*d)[ns]);
    }
    (*d)[value_key] = value;
  }

  if (notify_task)
  {
    auto updates = computeParamUpdates(reg_manager_->getParamSubscribers(), key, value, caller_id);
    if (!updates.empty())
    {
      notify_task(updates);
    }
  }
}

void ParamDictionary::deleteParam(const std::string& key, NotifyFunc notify_task)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);

  if (key == names::GLOBALNS)
  {
    throw std::out_of_range("cannot delete root of parameter tree");
  }

  auto ns_parts = splitString(key, '/');
  if (ns_parts.empty())
  {
    throw std::out_of_range("Parameter [" + key + "] is not set");
  }
  std::string value_key = ns_parts.back();
  ns_parts.pop_back();

  XmlRpc::XmlRpcValue* d = &parameters_;
  for (const auto& ns : ns_parts)
  {
    if (d->getType() != XmlRpc::XmlRpcValue::TypeStruct || !d->hasMember(ns))
    {
      throw std::out_of_range("Parameter [" + key + "] is not set");
    }
    d = &((*d)[ns]);
  }

  if (!d->hasMember(value_key))
  {
    throw std::out_of_range("Parameter [" + key + "] is not set");
  }

  // XmlRpcValue doesn't have a remove member method, so we rebuild without the key
  XmlRpc::XmlRpcValue new_d;
  bool has_other = false;
  for (auto it = d->begin(); it != d->end(); ++it)
  {
    if (it->first != value_key)
    {
      new_d[it->first] = it->second;
      has_other = true;
    }
  }
  if (has_other)
  {
    *d = new_d;
  }
  else
  {
    // Set to empty struct using a TypeInvalid placeholder (stripped by stripPlaceholders,
    // skipped by structToXml). Using TypeInvalid avoids reserving any user-visible key name.
    d->clear();
    (*d)["__init__"];
  }

  if (notify_task)
  {
    // Signal deletion with an empty struct {} (TypeInvalid sentinel, skipped by structToXml).
    XmlRpc::XmlRpcValue empty_val;
    empty_val["__empty__"] = XmlRpc::XmlRpcValue();
    auto updates = computeParamUpdates(reg_manager_->getParamSubscribers(), key, empty_val);
    if (!updates.empty())
    {
      notify_task(updates);
    }
  }
}

bool ParamDictionary::hasParam(const std::string& key)
{
  try
  {
    auto val = getParam(key);
    // Don't count internal placeholders as real params
    if (val.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
    {
      return false;
    }
    return true;
  }
  catch (const std::out_of_range&)
  {
    return false;
  }
}

XmlRpc::XmlRpcValue ParamDictionary::subscribeParam(
  const std::string& key,
  const std::string& caller_id,
  const std::string& caller_api)
{
  std::string k = key;
  if (k != "/")
  {
    k = names::canonicalizeName(k) + "/";
  }

  std::lock_guard<std::recursive_mutex> lock(lock_);
  XmlRpc::XmlRpcValue val;
  try
  {
    val = getParam(k);
  }
  catch (const std::out_of_range&)
  {
    // Parameter not set yet - return empty dict using a TypeInvalid placeholder
    val = XmlRpc::XmlRpcValue();
    val["__empty__"];
  }
  reg_manager_->registerParamSubscriber(k, caller_id, caller_api);
  return stripPlaceholders(val);
}

std::tuple<int, std::string, int> ParamDictionary::unsubscribeParam(
  const std::string& key,
  const std::string& caller_id,
  const std::string& caller_api)
{
  std::string k = key;
  if (k != "/")
  {
    k = names::canonicalizeName(k) + "/";
  }
  return reg_manager_->unregisterParamSubscriber(k, caller_id, caller_api);
}

}  // namespace rosmaster
