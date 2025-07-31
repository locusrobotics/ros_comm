/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/service_server_link.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/service_manager.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/file_log.h"

#include <boost/bind/bind.hpp>

#include <sstream>

namespace ros
{

ServiceServerLink::ServiceServerLink(const std::string& service_name, bool persistent, const std::string& request_md5sum,
                             const std::string& response_md5sum, const M_string& header_values, double timeout_sec)
: service_name_(service_name)
, persistent_(persistent)
, request_md5sum_(request_md5sum)
, response_md5sum_(response_md5sum)
, extra_outgoing_header_values_(header_values)
, header_written_(false)
, header_read_(false)
, dropped_(false)
, timeout_sec_(timeout_sec)
{
}

ServiceServerLink::~ServiceServerLink()
{
  ROS_ASSERT(connection_->isDropped());

  clearCalls();
}

void ServiceServerLink::cancelCall(const CallInfoPtr& info)
{
  boost::thread::id caller_thread_id;
  bool call_finished = false;

  {
    boost::mutex::scoped_lock lock(info->mutex_);
    info->finished_ = true;
    // If onResponse fires after we've returned from call() (due to this being cancelled), the pointer to resp_ will no longer be valid
    info->resp_ = nullptr;
    info->finished_condition_.notify_all();
    caller_thread_id = info->caller_thread_id_;
    call_finished = info->call_finished_;
  }

  if (boost::this_thread::get_id() != caller_thread_id)
  {
    while (!call_finished)
    {
      boost::this_thread::yield();

      boost::mutex::scoped_lock lock(info->mutex_);
      call_finished = info->call_finished_;
    }
  }
}

void ServiceServerLink::clearCalls()
{
  CallInfoPtr local_current;

  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);
    local_current = current_call_;
  }

  if (local_current)
  {
    cancelCall(local_current);
  }

  boost::mutex::scoped_lock lock(call_queue_mutex_);

  while (!call_queue_.empty())
  {
    CallInfoPtr info = call_queue_.front();

    cancelCall(info);

    call_queue_.pop();
  }
}

bool ServiceServerLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  connection_->addDropListener(boost::bind(&ServiceServerLink::onConnectionDropped, this, boost::placeholders::_1));
  connection_->setHeaderReceivedCallback(boost::bind(&ServiceServerLink::onHeaderReceived, this, boost::placeholders::_1, boost::placeholders::_2));

  M_string header;
  header["service"] = service_name_;
  header["md5sum"] = request_md5sum_;
  header["callerid"] = this_node::getName();
  header["persistent"] = persistent_ ? "1" : "0";
  header.insert(extra_outgoing_header_values_.begin(), extra_outgoing_header_values_.end());

  connection_->writeHeader(header, boost::bind(&ServiceServerLink::onHeaderWritten, this, boost::placeholders::_1));

  return true;
}

void ServiceServerLink::onHeaderWritten(const ConnectionPtr& conn)
{
  (void)conn;
  header_written_ = true;
}

bool ServiceServerLink::onHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  (void)conn;
  std::string md5sum, type;
  if (!header.getValue("md5sum", md5sum))
  {
    ROS_ERROR("TCPROS header from service server did not have required element: md5sum");
    return false;
  }

  bool empty = false;
  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);
    empty = call_queue_.empty();

    if (empty)
    {
      header_read_ = true;
    }
  }

  if (!empty)
  {
    processNextCall();

    header_read_ = true;
  }

  return true;
}

void ServiceServerLink::onConnectionDropped(const ConnectionPtr& conn)
{
  ROS_ASSERT(conn == connection_);
  ROSCPP_LOG_DEBUG("Service client from [%s] for [%s] dropped", conn->getRemoteString().c_str(), service_name_.c_str());

  dropped_ = true;
  clearCalls();

  ServiceManager::instance()->removeServiceServerLink(shared_from_this());
}

void ServiceServerLink::onRequestWritten(const ConnectionPtr& conn)
{
  (void)conn;
  //ros::WallDuration(0.1).sleep();

  boost::mutex::scoped_lock lock(call_queue_mutex_);
  auto current_call = current_call_;
  lock.unlock();

  connection_->read(5, boost::bind(&ServiceServerLink::onResponseOkAndLength, this, current_call, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
}

void ServiceServerLink::onResponseOkAndLength(CallInfoPtr info, const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  (void)size;
  ROS_ASSERT(conn == connection_);
  ROS_ASSERT(size == 5);

  if (!success)
    return;

  uint8_t ok = buffer[0];
  uint32_t len = *((uint32_t*)(buffer.get() + 1));

  if (len > 1000000000)
  {
    ROS_ERROR("a message of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost.");
    conn->drop(Connection::Destructing);

    return;
  }

  {
    boost::mutex::scoped_lock lock(info->mutex_);
    if ( ok != 0 ) {
      info->success_ = true;
    } else {
      info->success_ = false;
    }
  }

  if (len > 0)
  {
    connection_->read(len, boost::bind(&ServiceServerLink::onResponse, this, info, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
  }
  else
  {
    onResponse(info, conn, boost::shared_array<uint8_t>(), 0, true);
  }
}

void ServiceServerLink::onResponse(CallInfoPtr info, const ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  (void)conn;
  ROS_ASSERT(conn == connection_);

  if (!success)
    return;

  {
    boost::mutex::scoped_lock queue_lock(info->mutex_);

    // If this message was cancelled, the resp_ object will no longer be pointing at a valid response object
    // (we reset it to null)
    if (info && info->success_ && !info->finished_ && info->resp_)
    {
      *(info->resp_) = SerializedMessage(buffer, size);
    }
    else
    {
      info->exception_string_ = std::string(reinterpret_cast<char*>(buffer.get()), size);
    }
  }

  callFinished(info);
}

void ServiceServerLink::callFinished(CallInfoPtr info)
{
  CallInfoPtr saved_call;
  ServiceServerLinkPtr self;
  {
    boost::mutex::scoped_lock finished_lock(info->mutex_);

    ROS_DEBUG_NAMED("superdebug", "Client to service [%s] call finished with success=[%s]", service_name_.c_str(), current_call_->success_ ? "true" : "false");

    info->finished_ = true;
    info->finished_condition_.notify_all();
    info->call_finished_ = true;
    saved_call = info;
  }

  {
    boost::mutex::scoped_lock queue_lock(call_queue_mutex_);

    // Don't want to reassign current_call_ if the call that was just completed was a previous (timed out) one
    if (saved_call == current_call_)
    {
      current_call_ = CallInfoPtr();
    }

    // If the call queue is empty here, we may be deleted as soon as we release these locks, so keep a shared pointer to ourselves until we return
    // ugly
    // jfaust TODO there's got to be a better way
    self = shared_from_this();
  }

  processNextCall();
}

void ServiceServerLink::processNextCall()
{
  bool empty = false;
  {
    boost::mutex::scoped_lock lock(call_queue_mutex_);

    if (current_call_)
    {
      return;
    }

    if (!call_queue_.empty())
    {
      ROS_DEBUG_NAMED("superdebug", "[%s] Client to service [%s] processing next service call", persistent_ ? "persistent" : "non-persistent", service_name_.c_str());

      current_call_ = call_queue_.front();
      call_queue_.pop();
    }
    else
    {
      empty = true;
    }
  }

  if (empty)
  {
    if (!persistent_)
    {
      ROS_DEBUG_NAMED("superdebug", "Dropping non-persistent client to service [%s]", service_name_.c_str());
      connection_->drop(Connection::Destructing);
    }
    else
    {
      ROS_DEBUG_NAMED("superdebug", "Keeping persistent client to service [%s]", service_name_.c_str());
    }
  }
  else
  {
    SerializedMessage request;

    {
      boost::mutex::scoped_lock lock(call_queue_mutex_);
      request = current_call_->req_;
    }

    connection_->write(request.buf, request.num_bytes, boost::bind(&ServiceServerLink::onRequestWritten, this, boost::placeholders::_1));
  }
}

bool ServiceServerLink::call(const SerializedMessage& req, SerializedMessage& resp)
{
  CallInfoPtr info(boost::make_shared<CallInfo>());
  info->req_ = req;
  info->resp_ = &resp;
  info->success_ = false;
  info->finished_ = false;
  info->call_finished_ = false;
  info->caller_thread_id_ = boost::this_thread::get_id();

  //ros::WallDuration(0.1).sleep();

  bool immediate = false;
  {
    if (connection_->isDropped())
    {
      ROSCPP_LOG_DEBUG("ServiceServerLink::call called on dropped connection for service [%s]", service_name_.c_str());
      boost::mutex::scoped_lock lock(info->mutex_);
      info->call_finished_ = true;
      return false;
    }

    boost::mutex::scoped_lock lock(call_queue_mutex_);

    if (call_queue_.empty() && header_written_ && header_read_)
    {
      immediate = true;
    }

    call_queue_.push(info);
  }

  if (immediate)
  {
    processNextCall();
  }

  auto status = boost::cv_status::no_timeout;

  {
    boost::mutex::scoped_lock lock(info->mutex_);

    while (!info->finished_)
    {
      if (timeout_sec_ >= 0.0)
      {
        status = info->finished_condition_.wait_for(lock, boost::chrono::duration<double>(timeout_sec_));

        if (status == boost::cv_status::timeout)
        {
          lock.unlock();

          cancelCall(info);

          // If we've timed out, we need to move on to the next call. If we don't, and the service server has crashed
          // or restarted, the receiver thread will never reset current_call_ and the next call will never be
          // processed.
          boost::mutex::scoped_lock lock(call_queue_mutex_);

          // If the receiver callback fired in between locks, then we might have already reassigned current_call_
          if (info == current_call_)
          {
            current_call_ = CallInfoPtr();
          }

          break;
        }
      }
      else
      {
        info->finished_condition_.wait(lock);
      }
    }
  }

  bool success = false;
  std::string exception_string;
  {
    boost::mutex::scoped_lock lock(info->mutex_);
    info->call_finished_ = true;
    success = info->success_;
    exception_string = info->exception_string_;
  }

  if (status == boost::cv_status::timeout)
  {
    ROS_WARN_STREAM("Service call to " << service_name_ << " timed out after " << timeout_sec_ << " seconds");
  }

  if (exception_string.length() > 0)
  {
    ROS_ERROR("Service call failed: service [%s] responded with an error: %s", service_name_.c_str(), exception_string.c_str());
  }

  return success && status != boost::cv_status::timeout;
}

bool ServiceServerLink::isValid() const
{
  return !dropped_;
}

} // namespace ros
