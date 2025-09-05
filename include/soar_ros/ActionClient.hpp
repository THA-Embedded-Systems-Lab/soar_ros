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

#ifndef SOAR_ROS__ACTION_CLIENT_HPP_
#define SOAR_ROS__ACTION_CLIENT_HPP_

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sml_Client.h>

#include "Interface.hpp"
#include "SafeQueue.hpp"

namespace soar_ros
{

template<typename ActionT,
  typename pGoalMsg = typename ActionT::Goal::SharedPtr,
  typename pFeedbackMsg = typename ActionT::Feedback::SharedPtr,
  typename wrappedResultMsg = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult
>
class ActionClient
  : public Output<pGoalMsg>,
  public Input<bool>,
  public Input<pFeedbackMsg>,
  public Input<wrappedResultMsg>,
  public Interface
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  explicit ActionClient(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & action_name)
  : m_pAgent(agent), m_node(node), m_topic(action_name), isRunning(true)
  {
    client_ptr_ = rclcpp_action::create_client<ActionT>(node, action_name);
    m_action_thread = std::thread(&ActionClient::run, this);
  }

  virtual ~ActionClient()
  {
    isRunning.store(false);
    if (m_action_thread.joinable()) {
      m_action_thread.join();
    }
  }

  /// @brief Send a goal to the action server from a parsed Soar goal
  ///
  /// All future inputs for Soar must be inserted to queues to match the Soar
  /// processing cycle.
  /// @param soar_goal The goal message parsed from Soar
  void send_goal_from_soar(const pGoalMsg & soar_goal)
  {
    RCLCPP_INFO(m_node->get_logger(), "Sending goal to action server: %s", m_topic.c_str());

    auto options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

    // Goal response callback
    options.goal_response_callback = [this](std::shared_ptr<GoalHandle> goal_handle) {
        if (!goal_handle) {
          this->template Input<bool>::m_r2sQueue.push(false);
        } else {
          this->template Input<bool>::m_r2sQueue.push(true);
        }
      };

    // Feedback callback
    options.feedback_callback = [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        // Create a non-const copy for the queue
        auto feedback_copy = std::make_shared<typename ActionT::Feedback>(*feedback);
        this->template Input<pFeedbackMsg>::m_r2sQueue.push(feedback_copy);
      };

    // Result callback
    options.result_callback = [this](const wrappedResultMsg & wrapped) {
        this->template Input<wrappedResultMsg>::m_r2sQueue.push(wrapped);
      };

    client_ptr_->async_send_goal(*soar_goal, options);
  }

  // Interface implementation
  std::string getTopic() override {return m_topic;}
  sml::Agent * getAgent() override {return m_pAgent;}

protected:
  using Input<pFeedbackMsg>::parse;
  using Input<wrappedResultMsg>::parse;
  using Input<bool>::parse;

  virtual pGoalMsg parse(sml::Identifier * id) = 0;
  virtual void parse(pFeedbackMsg msg) = 0;
  virtual void parse(wrappedResultMsg msg) = 0;

  void parse(bool msg) override
  {
    sml::Identifier * il = this->getAgent()->GetInputLink();
    sml::Identifier * ros_action_id = il->CreateIdWME(this->m_topic.c_str());
    ros_action_id->CreateStringWME("status", msg ? "accepted" : "rejected");
  }

private:
  sml::Agent * m_pAgent;
  rclcpp::Node::SharedPtr m_node;
  std::string m_topic;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_ptr_;

  std::thread m_action_thread;
  std::atomic<bool> isRunning;

  // Background thread to process goals
  void run()
  {
    while (isRunning.load()) {
      // Wait for action server
      if (!client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for action server");
          isRunning.store(false);
          break;
        }
        continue;
      }

      // Process any pending goals
      auto goal = this->template Output<pGoalMsg>::m_s2rQueue.tryPop();
      if (goal.has_value()) {
        send_goal_from_soar(goal.value());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

}  // namespace soar_ros

#endif  // SOAR_ROS__ACTION_CLIENT_HPP_
