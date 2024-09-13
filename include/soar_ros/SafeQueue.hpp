// Copyright 2024 Moritz Schmidt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SOAR_ROS__SAFEQUEUE_HPP_
#define SOAR_ROS__SAFEQUEUE_HPP_

/**
 * @file SafeQueue.hpp
 * @brief Implementation of a thread safe queue.
 * @date 2023-12-08
 *
 * https://www.geeksforgeeks.org/implement-thread-safe-queue-in-c/
 *
 */

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <optional>

namespace soar_ros
{


template<typename T>
class SafeQueue
{
private:
  // Underlying queue
  std::queue<T> m_queue;

  // mutex for thread synchronization
  std::mutex m_mutex;

  // Condition variable for signaling
  std::condition_variable m_cond;

public:
  // Pushes an element to the queue
  void push(T item)
  {
    // Acquire lock
    std::unique_lock<std::mutex> lock(m_mutex);

    // Add item
    m_queue.push(item);

    // Notify one thread that
    // is waiting
    m_cond.notify_one();
  }

  T pop()
  {
    // acquire lock
    std::unique_lock<std::mutex> lock(m_mutex);

    // wait until queue is not empty
    m_cond.wait(
      lock,
      [this]()
      {return !m_queue.empty();});

    // retrieve item
    T item = m_queue.front();
    m_queue.pop();

    // return item
    return item;
  }

  std::optional<T> tryPop()
  {
    std::unique_lock<std::mutex> lock(m_mutex, std::try_to_lock);

    if (!lock.owns_lock()) {
      // Lock not acquired, return an empty optional
      return std::nullopt;
    }

    if (m_queue.empty()) {
      // Queue is empty, return an empty optional
      return std::nullopt;
    }

    T item = m_queue.front();
    m_queue.pop();
    return item;
  }
};
}

#endif  // SOAR_ROS__SAFEQUEUE_HPP_
