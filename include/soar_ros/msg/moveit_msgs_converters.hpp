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

/// @file moveit_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for a subset of moveit_msgs message types.
///
/// Optional include — requires moveit_msgs (and transitively sensor_msgs,
/// trajectory_msgs) in the consuming package.
///
/// Covered messages: MoveItErrorCodes, JointConstraint, RobotState (joint_state
/// field only — multi_dof_joint_state and attached_collision_objects are omitted
/// for simplicity).

#ifndef SOAR_ROS__MSG__MOVEIT_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__MOVEIT_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/sensor_msgs_converters.hpp"
#include "soar_ros/msg/trajectory_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── MoveItErrorCodes ──────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- val (int)  SUCCESS=1, FAILURE=-1, …

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const moveit_msgs::msg::MoveItErrorCodes &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("val", static_cast<int64_t>(msg.val));
        return id;
    }

    template <>
    inline moveit_msgs::msg::MoveItErrorCodes
    fromSoar<moveit_msgs::msg::MoveItErrorCodes>(sml::Identifier *id)
    {
        moveit_msgs::msg::MoveItErrorCodes msg;
        msg.val = static_cast<int32_t>(detail::getInt(id, "val"));
        return msg;
    }

    // ── JointConstraint ───────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- joint_name       (string)
    //     +-- position         (float)
    //     +-- tolerance_above  (float)
    //     +-- tolerance_below  (float)
    //     +-- weight           (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const moveit_msgs::msg::JointConstraint &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateStringWME("joint_name", msg.joint_name.c_str());
        id->CreateFloatWME("position", msg.position);
        id->CreateFloatWME("tolerance_above", msg.tolerance_above);
        id->CreateFloatWME("tolerance_below", msg.tolerance_below);
        id->CreateFloatWME("weight", msg.weight);
        return id;
    }

    template <>
    inline moveit_msgs::msg::JointConstraint
    fromSoar<moveit_msgs::msg::JointConstraint>(sml::Identifier *id)
    {
        moveit_msgs::msg::JointConstraint msg;
        msg.joint_name = detail::getString(id, "joint_name");
        msg.position = detail::getFloat(id, "position");
        msg.tolerance_above = detail::getFloat(id, "tolerance_above");
        msg.tolerance_below = detail::getFloat(id, "tolerance_below");
        msg.weight = detail::getFloat(id, "weight");
        return msg;
    }

    // ── RobotState (simplified) ───────────────────────────────────────────────────
    // Only joint_state is mapped. multi_dof_joint_state, attached_collision_objects
    // and is_diff are intentionally omitted.
    //
    // WME structure:
    //   <attr> (ID)
    //     +-- joint_state (ID)  → full JointState subtree
    //     +-- is_diff     (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const moveit_msgs::msg::RobotState &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "joint_state", msg.joint_state);
        id->CreateIntWME("is_diff", msg.is_diff ? 1 : 0);
        return id;
    }

    template <>
    inline moveit_msgs::msg::RobotState
    fromSoar<moveit_msgs::msg::RobotState>(sml::Identifier *id)
    {
        moveit_msgs::msg::RobotState msg;
        auto *js = detail::getChild(id, "joint_state");
        if (js)
            msg.joint_state = fromSoar<sensor_msgs::msg::JointState>(js);
        msg.is_diff = detail::getInt(id, "is_diff") != 0;
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__MOVEIT_MSGS_CONVERTERS_HPP_
