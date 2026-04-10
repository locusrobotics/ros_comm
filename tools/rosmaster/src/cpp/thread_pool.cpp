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

#include "rosmaster/thread_pool.h"

#include <chrono>
#include <iostream>

namespace rosmaster
{

MarkedThreadPool::MarkedThreadPool(int num_threads)
{
  for (int i = 0; i < num_threads; ++i)
  {
    threads_.emplace_back(&MarkedThreadPool::workerLoop, this);
  }
}

MarkedThreadPool::~MarkedThreadPool()
{
  joinAll(false);
}

void MarkedThreadPool::queueTask(const std::string& marker, TaskFunc task)
{
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    if (!accepting_)
    {
      return;
    }
    tasks_.push_back({marker, std::move(task)});
  }
  task_cv_.notify_one();
}

bool MarkedThreadPool::getNextTaskLocked(Task& out)
{
  for (auto it = tasks_.begin(); it != tasks_.end(); ++it)
  {
    if (it->marker.empty() || active_markers_.find(it->marker) == active_markers_.end())
    {
      out = std::move(*it);
      tasks_.erase(it);
      if (!out.marker.empty())
      {
        active_markers_.insert(out.marker);
      }
      return true;
    }
  }
  return false;
}

bool MarkedThreadPool::hasRunnableTask() const
{
  for (const auto& t : tasks_)
  {
    if (t.marker.empty() || active_markers_.find(t.marker) == active_markers_.end())
    {
      return true;
    }
  }
  return false;
}

void MarkedThreadPool::removeMarker(const std::string& marker)
{
  if (marker.empty())
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    active_markers_.erase(marker);
  }
  task_cv_.notify_all();
}

void MarkedThreadPool::workerLoop()
{
  while (true)
  {
    Task task;
    bool got_task = false;
    {
      std::unique_lock<std::mutex> lock(task_mutex_);
      task_cv_.wait(lock, [this] {
        return stopping_ || hasRunnableTask();
      });
      if (stopping_)
      {
        break;
      }
      got_task = getNextTaskLocked(task);
    }
    if (got_task)
    {
      try
      {
        task.func();
      }
      catch (const std::exception& e)
      {
        std::cerr << "[rosmaster.threadpool] Error: " << e.what() << std::endl;
      }
      catch (...)
      {
        std::cerr << "[rosmaster.threadpool] Unknown error in task" << std::endl;
      }
      removeMarker(task.marker);
    }
  }
}

void MarkedThreadPool::joinAll(bool wait_for_tasks)
{
  // Reject new tasks immediately
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    accepting_ = false;
  }

  if (wait_for_tasks)
  {
    // Drain queued tasks (no new tasks can arrive since accepting_ is false)
    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (tasks_.empty())
        {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    stopping_ = true;
  }
  task_cv_.notify_all();

  for (auto& t : threads_)
  {
    if (t.joinable())
    {
      t.join();
    }
  }
  threads_.clear();
}

}  // namespace rosmaster
