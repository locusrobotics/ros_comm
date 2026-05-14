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

#ifndef ROSMASTER_VALIDATORS_H
#define ROSMASTER_VALIDATORS_H

#include <functional>
#include <stdexcept>
#include <string>

namespace rosmaster
{

/// Exception thrown when a parameter fails validation
class ParameterInvalid : public std::runtime_error
{
public:
  explicit ParameterInvalid(const std::string& msg) : std::runtime_error(msg) {}
};

/// Validator function: takes (param_value, caller_id) returns canonicalized value
using ValidatorFunc = std::function<std::string(const std::string&, const std::string&)>;

/// Validate and resolve a ROS name
std::string validNameResolve(const std::string& param_name, const std::string& param_value,
                              const std::string& caller_id);

/// Validate a ROS name without resolving
std::string validNameUnresolved(const std::string& param_name, const std::string& param_value,
                                 const std::string& caller_id);

/// Validator: parameter must be non-empty string
ValidatorFunc nonEmptyStr(const std::string& param_name);

/// Validator: parameter must not be empty
ValidatorFunc nonEmpty(const std::string& param_name);

/// Validator: valid XML-RPC API URI (http:// or rosrpc://)
ValidatorFunc isApi(const std::string& param_name);

/// Validator: valid ROS topic name (resolved)
ValidatorFunc isTopic(const std::string& param_name);

/// Validator: valid ROS service name (resolved)
ValidatorFunc isService(const std::string& param_name);

/// Validator: empty string or valid resolved name
ValidatorFunc emptyOrValidName(const std::string& param_name);

/// Validator: valid resolved graph resource name
ValidatorFunc validName(const std::string& param_name, bool resolve = true);

/// Validator: valid message type name (package/Type)
ValidatorFunc validTypeName(const std::string& param_name);

}  // namespace rosmaster

#endif  // ROSMASTER_VALIDATORS_H
