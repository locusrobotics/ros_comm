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

#ifndef ROSMASTER_THREAD_POOL_H
#define ROSMASTER_THREAD_POOL_H

#include <condition_variable>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace rosmaster
{

/// A thread pool with marker-based task grouping.
/// Only one task with a given marker can execute at a time, preventing
/// slow I/O to a single node from consuming all threads.
class MarkedThreadPool
{
public:
  using TaskFunc = std::function<void()>;

  explicit MarkedThreadPool(int num_threads);
  ~MarkedThreadPool();

  /// Queue a task. marker is used to prevent concurrent tasks to the same target.
  /// An empty marker means no grouping constraint.
  void queueTask(const std::string& marker, TaskFunc task);

  /// Shut down the pool. If wait_for_tasks is true, drains the queue before
  /// stopping workers. Always joins all threads before returning.
  void joinAll(bool wait_for_tasks = true);

private:
  struct Task
  {
    std::string marker;
    TaskFunc func;
  };

  void workerLoop();
  bool getNextTaskLocked(Task& out);  // caller must hold task_mutex_
  bool hasRunnableTask() const;       // caller must hold task_mutex_
  void removeMarker(const std::string& marker);

  std::vector<std::thread> threads_;
  std::mutex task_mutex_;
  std::condition_variable task_cv_;
  std::vector<Task> tasks_;
  std::set<std::string> active_markers_;
  bool stopping_{false};    // protected by task_mutex_
  bool accepting_{true};    // rejects new tasks when false; protected by task_mutex_
};

}  // namespace rosmaster

#endif  // ROSMASTER_THREAD_POOL_H
