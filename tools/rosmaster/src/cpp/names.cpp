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

#include "rosmaster/names.h"

#include <algorithm>
#include <regex>
#include <sstream>
#include <vector>

namespace rosmaster
{
namespace names
{

namespace
{
std::vector<std::string> splitName(const std::string& name, char sep)
{
  std::vector<std::string> parts;
  std::istringstream stream(name);
  std::string part;
  while (std::getline(stream, part, sep))
  {
    if (!part.empty())
    {
      parts.push_back(part);
    }
  }
  return parts;
}
}  // anonymous namespace

std::string getNamespace(const std::string& name)
{
  if (name.empty())
  {
    return GLOBALNS;
  }
  std::string n = name;
  if (n.back() == SEP)
  {
    n = n.substr(0, n.size() - 1);
  }
  auto pos = n.rfind(SEP);
  if (pos == std::string::npos)
  {
    return GLOBALNS;
  }
  return n.substr(0, pos + 1);
}

std::string nsJoin(const std::string& ns, const std::string& name)
{
  if (isPrivate(name) || isGlobal(name))
  {
    return name;
  }
  if (ns.empty())
  {
    return name;
  }
  if (ns.size() == 1 && ns[0] == PRIV_NAME)
  {
    return std::string(1, PRIV_NAME) + name;
  }
  if (ns.back() == SEP)
  {
    return ns + name;
  }
  return ns + SEP + name;
}

std::string canonicalizeName(const std::string& name)
{
  if (name.empty() || name == GLOBALNS)
  {
    return name;
  }
  auto parts = splitName(name, SEP);
  std::string result;
  if (name[0] == SEP)
  {
    result = "/";
  }
  for (size_t i = 0; i < parts.size(); ++i)
  {
    if (i > 0)
    {
      result += "/";
    }
    result += parts[i];
  }
  return result;
}

std::string resolveName(const std::string& name, const std::string& ns)
{
  if (name.empty())
  {
    return getNamespace(ns);
  }
  std::string canonical = canonicalizeName(name);
  if (canonical[0] == SEP)
  {
    return canonical;
  }
  if (isPrivate(canonical))
  {
    return canonicalizeName(ns + SEP + canonical.substr(1));
  }
  // relative name
  return getNamespace(ns) + canonical;
}

bool isLegalName(const std::string& name)
{
  if (name.empty())
  {
    return true;  // empty resolves to namespace
  }
  static const std::regex legal_chars("^[~/A-Za-z][\\w/]*$");
  if (!std::regex_match(name, legal_chars))
  {
    return false;
  }
  return name.find("//") == std::string::npos;
}

std::string makeGlobalNs(const std::string& name)
{
  if (isPrivate(name))
  {
    return {};  // cannot turn private into global
  }
  std::string result = name;
  if (!isGlobal(result))
  {
    result = SEP + result;
  }
  if (result.back() != SEP)
  {
    result += SEP;
  }
  return result;
}

}  // namespace names
}  // namespace rosmaster
