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

#include "rosmaster/validators.h"
#include "rosmaster/names.h"

#include <algorithm>

namespace rosmaster
{

std::string validNameResolve(const std::string& param_name, const std::string& param_value,
                              const std::string& caller_id)
{
  if (param_value.empty())
  {
    throw ParameterInvalid("ERROR: parameter [" + param_name + "] must be a non-empty string");
  }
  if (param_value.find(':') != std::string::npos || param_value.find(' ') != std::string::npos)
  {
    throw ParameterInvalid("ERROR: parameter [" + param_name + "] contains illegal chars");
  }
  return names::resolveName(param_value, caller_id);
}

std::string validNameUnresolved(const std::string& param_name, const std::string& param_value,
                                 const std::string& caller_id)
{
  if (param_value.empty())
  {
    throw ParameterInvalid("ERROR: parameter [" + param_name + "] must be a non-empty string");
  }
  if (param_value.find(':') != std::string::npos || param_value.find(' ') != std::string::npos)
  {
    throw ParameterInvalid("ERROR: parameter [" + param_name + "] contains illegal chars");
  }
  return param_value;
}

ValidatorFunc nonEmptyStr(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& /*context*/) -> std::string {
    if (param.empty())
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] must be specified and non-empty");
    }
    return param;
  };
}

ValidatorFunc nonEmpty(const std::string& param_name)
{
  // Delegates to nonEmptyStr; both exist for parity with Python rosmaster validators.
  return nonEmptyStr(param_name);
}

ValidatorFunc isApi(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& /*context*/) -> std::string {
    if (param.empty())
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] is not an XMLRPC URI");
    }
    if (param.substr(0, 7) != "http://" && param.substr(0, 9) != "rosrpc://")
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] is not an RPC URI");
    }
    return param;
  };
}

ValidatorFunc isTopic(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& caller_id) -> std::string {
    std::string v = validNameResolve(param_name, param, caller_id);
    if (param == "/")
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] cannot be the global namespace");
    }
    return v;
  };
}

ValidatorFunc isService(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& caller_id) -> std::string {
    std::string v = validNameResolve(param_name, param, caller_id);
    if (param == "/")
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] cannot be the global namespace");
    }
    return v;
  };
}

ValidatorFunc emptyOrValidName(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& caller_id) -> std::string {
    if (param.empty())
    {
      return "";
    }
    return names::resolveName(param, caller_id);
  };
}

ValidatorFunc validName(const std::string& param_name, bool resolve)
{
  return [param_name, resolve](const std::string& param, const std::string& caller_id) -> std::string {
    if (resolve)
    {
      return validNameResolve(param_name, param, caller_id);
    }
    return validNameUnresolved(param_name, param, caller_id);
  };
}

ValidatorFunc validTypeName(const std::string& param_name)
{
  return [param_name](const std::string& param, const std::string& /*caller_id*/) -> std::string {
    if (param == names::ANYTYPE)
    {
      return param;
    }
    if (param.empty())
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name + "] must be a non-empty string");
    }
    if (std::count(param.begin(), param.end(), '/') != 1)
    {
      throw ParameterInvalid("ERROR: parameter [" + param_name +
                              "] is not a valid package resource name");
    }
    return param;
  };
}

}  // namespace rosmaster
