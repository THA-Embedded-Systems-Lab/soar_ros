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

/// @file action_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for action_msgs message types.
///
/// Optional include — requires action_msgs and unique_identifier_msgs in the
/// consuming package.
///
/// UUID (unique_identifier_msgs::msg::UUID) is stored as 16 indexed int WMEs:
/// uuid_0 … uuid_15.

#ifndef SOAR_ROS__MSG__ACTION_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__ACTION_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include "soar_ros/msg/builtin_interfaces_converters.hpp"
#include "soar_ros/msg/detail.hpp"

namespace soar_ros::msg
{

    // ── GoalInfo ──────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- goal_id (ID)  → uuid_0 … uuid_15 (int, one byte each)
    //     +-- stamp   (ID)  → sec, nanosec

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const action_msgs::msg::GoalInfo &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        auto *uuid_id = id->CreateIdWME("goal_id");
        for (int i = 0; i < 16; ++i)
            uuid_id->CreateIntWME(("uuid_" + std::to_string(i)).c_str(),
                                  static_cast<int64_t>(msg.goal_id.uuid[i]));
        toSoar(id, "stamp", msg.stamp);
        return id;
    }

    template <>
    inline action_msgs::msg::GoalInfo
    fromSoar<action_msgs::msg::GoalInfo>(sml::Identifier *id)
    {
        action_msgs::msg::GoalInfo msg;
        auto *uuid_id = detail::getChild(id, "goal_id");
        if (uuid_id)
        {
            for (int i = 0; i < 16; ++i)
                msg.goal_id.uuid[i] =
                    static_cast<uint8_t>(detail::getInt(uuid_id, ("uuid_" + std::to_string(i)).c_str()));
        }
        auto *stamp = detail::getChild(id, "stamp");
        if (stamp)
            msg.stamp = fromSoar<builtin_interfaces::msg::Time>(stamp);
        return msg;
    }

    // ── GoalStatus ────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- goal_info (ID)
    //     +-- status    (int)  STATUS_UNKNOWN=0, ACCEPTED=1, EXECUTING=2, …

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const action_msgs::msg::GoalStatus &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "goal_info", msg.goal_info);
        id->CreateIntWME("status", static_cast<int64_t>(msg.status));
        return id;
    }

    template <>
    inline action_msgs::msg::GoalStatus
    fromSoar<action_msgs::msg::GoalStatus>(sml::Identifier *id)
    {
        action_msgs::msg::GoalStatus msg;
        auto *gi = detail::getChild(id, "goal_info");
        if (gi)
            msg.goal_info = fromSoar<action_msgs::msg::GoalInfo>(gi);
        msg.status = static_cast<int8_t>(detail::getInt(id, "status"));
        return msg;
    }

    // ── GoalStatusArray ───────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- status_list_count  (int)
    //     +-- status_list_0, …   (ID)  → goal_info, status

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const action_msgs::msg::GoalStatusArray &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("status_list_count", static_cast<int64_t>(msg.status_list.size()));
        for (std::size_t i = 0; i < msg.status_list.size(); ++i)
            toSoar(id, ("status_list_" + std::to_string(i)).c_str(), msg.status_list[i]);
        return id;
    }

    template <>
    inline action_msgs::msg::GoalStatusArray
    fromSoar<action_msgs::msg::GoalStatusArray>(sml::Identifier *id)
    {
        action_msgs::msg::GoalStatusArray msg;
        int64_t count = detail::getInt(id, "status_list_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("status_list_" + std::to_string(i)).c_str());
            if (child)
                msg.status_list.push_back(fromSoar<action_msgs::msg::GoalStatus>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__ACTION_MSGS_CONVERTERS_HPP_
