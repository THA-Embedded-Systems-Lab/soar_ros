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

/// @file trajectory_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for trajectory_msgs message types.
///
/// Optional include — requires trajectory_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__TRAJECTORY_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__TRAJECTORY_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "soar_ros/msg/builtin_interfaces_converters.hpp"
#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── JointTrajectoryPoint ──────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- positions_count      (int)
    //     +-- positions_0, …       (float)
    //     +-- velocities_count     (int)
    //     +-- velocities_0, …      (float)
    //     +-- accelerations_count  (int)
    //     +-- accelerations_0, …   (float)
    //     +-- effort_count         (int)
    //     +-- effort_0, …          (float)
    //     +-- time_from_start      (ID)  → sec, nanosec

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const trajectory_msgs::msg::JointTrajectoryPoint &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        detail::writeNumericArray(id, "positions", msg.positions);
        detail::writeNumericArray(id, "velocities", msg.velocities);
        detail::writeNumericArray(id, "accelerations", msg.accelerations);
        detail::writeNumericArray(id, "effort", msg.effort);
        toSoar(id, "time_from_start", msg.time_from_start);
        return id;
    }

    template <>
    inline trajectory_msgs::msg::JointTrajectoryPoint
    fromSoar<trajectory_msgs::msg::JointTrajectoryPoint>(sml::Identifier *id)
    {
        trajectory_msgs::msg::JointTrajectoryPoint msg;
        msg.positions = detail::readNumericArray<double>(id, "positions");
        msg.velocities = detail::readNumericArray<double>(id, "velocities");
        msg.accelerations = detail::readNumericArray<double>(id, "accelerations");
        msg.effort = detail::readNumericArray<double>(id, "effort");
        auto *tfs = detail::getChild(id, "time_from_start");
        if (tfs)
            msg.time_from_start = fromSoar<builtin_interfaces::msg::Duration>(tfs);
        return msg;
    }

    // ── JointTrajectory ───────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header              (ID)
    //     +-- joint_names_count   (int)
    //     +-- joint_names_0, …    (string)
    //     +-- points_count        (int)
    //     +-- points_0, …         (ID)  → JointTrajectoryPoint subtree

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const trajectory_msgs::msg::JointTrajectory &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        detail::writeStringArray(id, "joint_names", msg.joint_names);
        id->CreateIntWME("points_count", static_cast<int64_t>(msg.points.size()));
        for (std::size_t i = 0; i < msg.points.size(); ++i)
            toSoar(id, ("points_" + std::to_string(i)).c_str(), msg.points[i]);
        return id;
    }

    template <>
    inline trajectory_msgs::msg::JointTrajectory
    fromSoar<trajectory_msgs::msg::JointTrajectory>(sml::Identifier *id)
    {
        trajectory_msgs::msg::JointTrajectory msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.joint_names = detail::readStringArray(id, "joint_names");
        int64_t count = detail::getInt(id, "points_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("points_" + std::to_string(i)).c_str());
            if (child)
                msg.points.push_back(
                    fromSoar<trajectory_msgs::msg::JointTrajectoryPoint>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__TRAJECTORY_MSGS_CONVERTERS_HPP_
