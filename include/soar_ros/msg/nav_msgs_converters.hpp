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

/// @file nav_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for nav_msgs message types.
///
/// Optional include — requires nav_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__NAV_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__NAV_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "soar_ros/msg/builtin_interfaces_converters.hpp"
#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/geometry_msgs_converters.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── MapMetaData ───────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- map_load_time (ID)  → sec, nanosec
    //     +-- resolution    (float)
    //     +-- width         (int)
    //     +-- height        (int)
    //     +-- origin        (ID)  → position, orientation

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const nav_msgs::msg::MapMetaData &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "map_load_time", msg.map_load_time);
        id->CreateFloatWME("resolution", static_cast<double>(msg.resolution));
        id->CreateIntWME("width", static_cast<int64_t>(msg.width));
        id->CreateIntWME("height", static_cast<int64_t>(msg.height));
        toSoar(id, "origin", msg.origin);
        return id;
    }

    template <>
    inline nav_msgs::msg::MapMetaData
    fromSoar<nav_msgs::msg::MapMetaData>(sml::Identifier *id)
    {
        nav_msgs::msg::MapMetaData msg;
        auto *mlt = detail::getChild(id, "map_load_time");
        if (mlt)
            msg.map_load_time = fromSoar<builtin_interfaces::msg::Time>(mlt);
        msg.resolution = static_cast<float>(detail::getFloat(id, "resolution"));
        msg.width = static_cast<uint32_t>(detail::getInt(id, "width"));
        msg.height = static_cast<uint32_t>(detail::getInt(id, "height"));
        auto *origin = detail::getChild(id, "origin");
        if (origin)
            msg.origin = fromSoar<geometry_msgs::msg::Pose>(origin);
        return msg;
    }

    // ── Odometry ──────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header         (ID)
    //     +-- child_frame_id (string)
    //     +-- pose           (ID)  → pose (ID) + covariance_0 … covariance_35
    //     +-- twist          (ID)  → twist (ID) + covariance_0 … covariance_35

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const nav_msgs::msg::Odometry &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateStringWME("child_frame_id", msg.child_frame_id.c_str());
        toSoar(id, "pose", msg.pose);
        toSoar(id, "twist", msg.twist);
        return id;
    }

    template <>
    inline nav_msgs::msg::Odometry
    fromSoar<nav_msgs::msg::Odometry>(sml::Identifier *id)
    {
        nav_msgs::msg::Odometry msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.child_frame_id = detail::getString(id, "child_frame_id");
        auto *pose = detail::getChild(id, "pose");
        if (pose)
            msg.pose = fromSoar<geometry_msgs::msg::PoseWithCovariance>(pose);
        auto *twist = detail::getChild(id, "twist");
        if (twist)
            msg.twist = fromSoar<geometry_msgs::msg::TwistWithCovariance>(twist);
        return msg;
    }

    // ── Path ──────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header        (ID)
    //     +-- poses_count   (int)
    //     +-- poses_0, …    (ID)  → header, pose

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const nav_msgs::msg::Path &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateIntWME("poses_count", static_cast<int64_t>(msg.poses.size()));
        for (std::size_t i = 0; i < msg.poses.size(); ++i)
            toSoar(id, ("poses_" + std::to_string(i)).c_str(), msg.poses[i]);
        return id;
    }

    template <>
    inline nav_msgs::msg::Path
    fromSoar<nav_msgs::msg::Path>(sml::Identifier *id)
    {
        nav_msgs::msg::Path msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        int64_t count = detail::getInt(id, "poses_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("poses_" + std::to_string(i)).c_str());
            if (child)
                msg.poses.push_back(fromSoar<geometry_msgs::msg::PoseStamped>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__NAV_MSGS_CONVERTERS_HPP_
