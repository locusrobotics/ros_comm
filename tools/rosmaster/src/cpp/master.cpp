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

#include "rosmaster/master.h"
#include "rosmaster/master_handler.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rosmaster
{

namespace
{

/**
 * @brief Find a non-loopback local IPv4 address, mirroring rosgraph.network.get_local_address().
 *
 * Enumerates network interfaces via getifaddrs() and returns the first IPv4 address that is
 * not on the loopback interface (127.x.x.x). Falls back to "127.0.0.1" if no such address
 * exists (e.g. on a machine with only a loopback interface).
 *
 * @return Non-loopback IPv4 address string, or "127.0.0.1" as a last resort.
 */
std::string getLocalAddress()
{
  std::string fallback = "127.0.0.1";

  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0 || ifaddr == nullptr)
  {
    return fallback;
  }

  std::string result;
  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next)
  {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
    {
      continue;
    }
    char buf[INET_ADDRSTRLEN] = {0};
    auto* sin = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
    inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf));

    // Skip loopback addresses (127.x.x.x)
    if (std::strncmp(buf, "127.", 4) == 0)
    {
      continue;
    }

    result = buf;
    break;
  }

  freeifaddrs(ifaddr);
  return result.empty() ? fallback : result;
}

}  // anonymous namespace

Master::Master(int port, int num_workers) : port_(port), num_workers_(num_workers)
{
}

Master::~Master()
{
  stop();
}

void Master::start()
{
  server_ = std::make_unique<XmlRpc::XmlRpcServer>();

  if (!server_->bindAndListen(port_))
  {
    throw std::runtime_error("Failed to bind ROS Master to port " + std::to_string(port_));
  }

  int actual_port = server_->get_port();

  handler_ = std::make_unique<ROSMasterHandler>(server_.get(), num_workers_);

  // Determine URI: mirror rosgraph.network.get_host_name() logic.
  // Priority: ROS_HOSTNAME > ROS_IP > gethostname() > non-loopback interface address.
  const char* ros_hostname = std::getenv("ROS_HOSTNAME");
  const char* ros_ip = std::getenv("ROS_IP");

  std::string host;
  if (ros_hostname && ros_hostname[0])
  {
    host = ros_hostname;
  }
  else if (ros_ip && ros_ip[0])
  {
    host = ros_ip;
  }
  else
  {
    char hostname_buf[256] = {0};
    if (gethostname(hostname_buf, sizeof(hostname_buf)) == 0)
    {
      host = hostname_buf;
    }
    // If gethostname() failed, returned empty, "localhost", or a loopback address,
    // fall back to a non-loopback local interface address (matching Python behaviour).
    if (host.empty() || host == "localhost" || host.substr(0, 4) == "127.")
    {
      host = getLocalAddress();
    }
  }

  uri_ = "http://" + host + ":" + std::to_string(actual_port) + "/";
  handler_->setUri(uri_);
}

bool Master::ok() const
{
  return handler_ && handler_->ok();
}

void Master::stop()
{
  if (handler_)
  {
    handler_->shutdown("Master.stop");
    handler_.reset();
  }
  if (server_)
  {
    server_->shutdown();
    server_.reset();
  }
}

const std::string& Master::uri() const
{
  return uri_;
}

void Master::spinOnce(double timeout_ms)
{
  if (server_)
  {
    server_->work(timeout_ms / 1000.0);
  }
}

void Master::interrupt()
{
  if (server_)
  {
    server_->exit();
  }
}

}  // namespace rosmaster
