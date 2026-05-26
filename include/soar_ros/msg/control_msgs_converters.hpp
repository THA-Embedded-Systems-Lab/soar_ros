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

/// @file control_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for control_msgs message types.
///
/// Optional include — requires control_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__CONTROL_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__CONTROL_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <control_msgs/msg/gripper_command.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── GripperCommand ────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- position   (float)
    //     +-- max_effort (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const control_msgs::msg::GripperCommand &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("position", msg.position);
        id->CreateFloatWME("max_effort", msg.max_effort);
        return id;
    }

    template <>
    inline control_msgs::msg::GripperCommand
    fromSoar<control_msgs::msg::GripperCommand>(sml::Identifier *id)
    {
        control_msgs::msg::GripperCommand msg;
        msg.position = detail::getFloat(id, "position");
        msg.max_effort = detail::getFloat(id, "max_effort");
        return msg;
    }

    // ── JointTolerance ────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- name         (string)
    //     +-- position     (float)
    //     +-- velocity     (float)
    //     +-- acceleration (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const control_msgs::msg::JointTolerance &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateStringWME("name", msg.name.c_str());
        id->CreateFloatWME("position", msg.position);
        id->CreateFloatWME("velocity", msg.velocity);
        id->CreateFloatWME("acceleration", msg.acceleration);
        return id;
    }

    template <>
    inline control_msgs::msg::JointTolerance
    fromSoar<control_msgs::msg::JointTolerance>(sml::Identifier *id)
    {
        control_msgs::msg::JointTolerance msg;
        msg.name = detail::getString(id, "name");
        msg.position = detail::getFloat(id, "position");
        msg.velocity = detail::getFloat(id, "velocity");
        msg.acceleration = detail::getFloat(id, "acceleration");
        return msg;
    }

    // ── JointJog ──────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header              (ID)
    //     +-- joint_names_count   (int)
    //     +-- joint_names_0, …    (string)
    //     +-- displacements_count (int)
    //     +-- displacements_0, …  (float)
    //     +-- velocities_count    (int)
    //     +-- velocities_0, …     (float)
    //     +-- duration            (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const control_msgs::msg::JointJog &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        detail::writeStringArray(id, "joint_names", msg.joint_names);
        detail::writeNumericArray(id, "displacements", msg.displacements);
        detail::writeNumericArray(id, "velocities", msg.velocities);
        id->CreateFloatWME("duration", msg.duration);
        return id;
    }

    template <>
    inline control_msgs::msg::JointJog
    fromSoar<control_msgs::msg::JointJog>(sml::Identifier *id)
    {
        control_msgs::msg::JointJog msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.joint_names = detail::readStringArray(id, "joint_names");
        msg.displacements = detail::readNumericArray<double>(id, "displacements");
        msg.velocities = detail::readNumericArray<double>(id, "velocities");
        msg.duration = detail::getFloat(id, "duration");
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__CONTROL_MSGS_CONVERTERS_HPP_
