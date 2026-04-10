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

#ifndef ROSMASTER_NAMES_H
#define ROSMASTER_NAMES_H

#include <string>
#include <regex>

namespace rosmaster
{
namespace names
{

const char SEP = '/';
const std::string GLOBALNS = "/";
const char PRIV_NAME = '~';
const std::string ANYTYPE = "*";

/// Test if name is a global graph resource name (starts with /)
inline bool isGlobal(const std::string& name)
{
  return !name.empty() && name[0] == SEP;
}

/// Test if name is a private graph resource name (starts with ~)
inline bool isPrivate(const std::string& name)
{
  return !name.empty() && name[0] == PRIV_NAME;
}

/// Get the namespace of a name. Returned with trailing slash.
std::string getNamespace(const std::string& name);

/// Join a namespace and name. If name is ~private or /global, returns name unchanged.
std::string nsJoin(const std::string& ns, const std::string& name);

/// Put name in canonical form: remove double slashes, strip trailing slash
std::string canonicalizeName(const std::string& name);

/// Resolve a ROS name to its global, canonical form
std::string resolveName(const std::string& name, const std::string& ns);

/// Check if name is a legal ROS graph resource name
bool isLegalName(const std::string& name);

/// Convert name to a global name with trailing separator
std::string makeGlobalNs(const std::string& name);

}  // namespace names
}  // namespace rosmaster

#endif  // ROSMASTER_NAMES_H
