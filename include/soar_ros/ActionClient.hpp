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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

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

  /// @param min_send_interval Minimum wall time between one goal finishing and
  ///        the next being sent. 0 (default) sends as fast as Soar asks; a
  ///        non-zero value throttles a Soar rule that re-requests an action on
  ///        every idle cycle (e.g. detect-workspace.soar) down to a poll.
  explicit ActionClient(
    sml::Agent * agent,
    rclcpp::Node::SharedPtr node,
    const std::string & action_name,
    std::chrono::milliseconds min_send_interval = std::chrono::milliseconds(0))
  : m_pAgent(agent), m_node(node), m_topic(action_name),
    m_min_interval(min_send_interval), isRunning(true)
  {
    // Two problems this avoids:
    //  1. rclcpp_action::Client is not safe to call async_send_goal() on from
    //     one thread while an executor delivers its responses on another -- a
    //     fast local server can answer before async_send_goal() finished
    //     registering the goal, and rclcpp_action then logs "unknown goal
    //     response, ignoring..." and drops the result. The client is put in
    //     its own callback group which this class spins on its own thread, so
    //     sending and response handling are on one thread and never race. The
    //     group is deliberately NOT added to the node's executor -- the rest of
    //     the agent is untouched.
    //  2. Soar can queue goals faster than it consumes results, and a fresh
    //     goal tears down the previous result on the input-link before the
    //     agent has read it (see e.g. DetectionActionClient::parse). Only one
    //     goal is kept in flight; the rest wait in the Soar output queue.
    m_cbg = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    client_ptr_ = rclcpp_action::create_client<ActionT>(node, action_name, m_cbg);
    m_exec.add_callback_group(m_cbg, node->get_node_base_interface());
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
    RCLCPP_DEBUG(m_node->get_logger(), "Sending goal to action server: %s", m_topic.c_str());

    auto options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

    // Goal response callback
    options.goal_response_callback = [this](std::shared_ptr<GoalHandle> goal_handle) {
        if (!goal_handle) {
          this->template Input<bool>::m_r2sQueue.push(false);
          m_goal_in_flight = false;  // rejected -> free to send the next
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
        m_goal_in_flight = false;
        m_last_goal_done = std::chrono::steady_clock::now();
      };

    m_goal_in_flight = true;
    m_goal_sent_at = std::chrono::steady_clock::now();
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

  std::chrono::milliseconds m_min_interval;
  std::atomic<bool> isRunning;
  rclcpp::CallbackGroup::SharedPtr m_cbg;
  rclcpp::executors::SingleThreadedExecutor m_exec;
  std::thread m_action_thread;
  bool m_goal_in_flight{false};  // only ever touched from m_action_thread
  std::chrono::steady_clock::time_point m_goal_sent_at{};
  std::chrono::steady_clock::time_point m_last_goal_done{};

  // Own thread: process this client's own responses, then send the next
  // queued goal. Everything action-client happens here, so async_send_goal()
  // and the response handlers never run concurrently. Idles at 10 Hz (an
  // action goal takes far longer than that) and only tightens the loop while
  // a goal is outstanding, so an idle client barely costs anything.
  void run()
  {
    while (isRunning.load()) {
      m_exec.spin_some(std::chrono::milliseconds(20));
      if (rclcpp::ok()) {
        pollAndSend();
      }
      std::this_thread::sleep_for(
        m_goal_in_flight ? std::chrono::milliseconds(20) : std::chrono::milliseconds(100));
    }
  }

  // Send the next queued goal, but only when the server is up, no goal is
  // still outstanding, and at least m_min_interval has passed since the last
  // one finished. A goal whose result never came back (server died, dropped
  // response) is abandoned after 60 s so the client cannot wedge.
  void pollAndSend()
  {
    if (m_goal_in_flight) {
      if (std::chrono::steady_clock::now() - m_goal_sent_at < std::chrono::seconds(60)) {
        return;
      }
      RCLCPP_WARN(
        m_node->get_logger(), "Action '%s': goal result timed out, abandoning it.",
        m_topic.c_str());
      m_goal_in_flight = false;
    }
    if (m_min_interval.count() > 0 &&
      std::chrono::steady_clock::now() - m_last_goal_done < m_min_interval)
    {
      return;
    }
    if (!client_ptr_->action_server_is_ready()) {
      return;
    }
    auto goal = this->template Output<pGoalMsg>::m_s2rQueue.tryPop();
    if (goal.has_value()) {
      send_goal_from_soar(goal.value());
    }
  }
};

}  // namespace soar_ros

#endif  // SOAR_ROS__ACTION_CLIENT_HPP_
