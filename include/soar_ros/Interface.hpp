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

#ifndef SOAR_ROS__INTERFACE_HPP_
#define SOAR_ROS__INTERFACE_HPP_


#include <string>

#include "SafeQueue.hpp"

#include "sml_Client.h"

namespace soar_ros
{
class Interface
{
public:
  /// @brief Get the topic of the subscriber
  /// @return
  virtual std::string getTopic() = 0;

  /// @brief Get the agent of the current interface
  /// @return
  virtual sml::Agent * getAgent() = 0;
};

class OutputBase
{
public:
  /// @brief This function is called to process an output link WME and
  /// push it to a queue.
  /// @param id
  virtual void process_s2r(sml::Identifier * id) = 0;
};

class InputBase
{
public:
  /// @brief This function reads the queue and calls the parse function of
  /// the input to create new WM elements via parse function, cf. Input
  virtual void process_r2s() = 0;
};

template<typename T>
class Output : public OutputBase
{
protected:
  SafeQueue<T> m_s2rQueue;

public:
  Output() {}
  ~Output() {}

  void process_s2r(sml::Identifier * id) override
  {
    m_s2rQueue.push(parse(id));
  }

  /// @brief Parse Soar working memory structure to a ROS message
  /// @param id
  /// @return
  virtual T parse(sml::Identifier * id) = 0;
};

template<typename T>
class Input : public InputBase
{
protected:
  /// @brief
  SafeQueue<T> m_r2sQueue;

public:
  Input() {}
  ~Input() {}

  /// @brief This function must attach the ROS message T from m_r2sQueue to the
  /// Soar input link.
  ///
  /// Soar usually requires a `pAgent->Commit()` to send the changes, but this is
  /// handled inside the SoarRunner::processInput() call.
  ///
  /// The function is called inside the Interface::Input::process_r2s() function.
  ///
  /// @param msg The ROS2 message.
  virtual void parse(T msg) = 0;

  /// @brief
  void process_r2s() override
  {
    auto res = this->m_r2sQueue.tryPop();
    if (res.has_value()) {
      this->parse(res.value());
    }
  }
};

}  // namespace soar_ros
#endif  // SOAR_ROS__INTERFACE_HPP_
