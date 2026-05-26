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

/// @file visualization_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for visualization_msgs message types.
///
/// Optional include — requires visualization_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__VISUALIZATION_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__VISUALIZATION_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "soar_ros/msg/builtin_interfaces_converters.hpp"
#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/geometry_msgs_converters.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── Marker ────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header        (ID)
    //     +-- ns            (string)
    //     +-- id            (int)
    //     +-- type          (int)   ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, …
    //     +-- action        (int)   ADD=0, MODIFY=0, DELETE=2, DELETEALL=3
    //     +-- pose          (ID)    → position, orientation
    //     +-- scale         (ID)    → x, y, z
    //     +-- color         (ID)    → r, g, b, a
    //     +-- lifetime      (ID)    → sec, nanosec
    //     +-- frame_locked  (int)   1 = true
    //     +-- text          (string)
    //     +-- mesh_resource (string)
    //     +-- points_count  (int)   array of Point
    //     +-- points_0, …   (ID)    → x, y, z
    //     +-- colors_count  (int)   parallel array of ColorRGBA
    //     +-- colors_0, …   (ID)    → r, g, b, a

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const visualization_msgs::msg::Marker &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateStringWME("ns", msg.ns.c_str());
        id->CreateIntWME("id", static_cast<int64_t>(msg.id));
        id->CreateIntWME("type", static_cast<int64_t>(msg.type));
        id->CreateIntWME("action", static_cast<int64_t>(msg.action));
        toSoar(id, "pose", msg.pose);
        toSoar(id, "scale", msg.scale);
        toSoar(id, "color", msg.color);
        toSoar(id, "lifetime", msg.lifetime);
        id->CreateIntWME("frame_locked", msg.frame_locked ? 1 : 0);
        id->CreateStringWME("text", msg.text.c_str());
        id->CreateStringWME("mesh_resource", msg.mesh_resource.c_str());
        id->CreateIntWME("points_count", static_cast<int64_t>(msg.points.size()));
        for (std::size_t i = 0; i < msg.points.size(); ++i)
            toSoar(id, ("points_" + std::to_string(i)).c_str(), msg.points[i]);
        id->CreateIntWME("colors_count", static_cast<int64_t>(msg.colors.size()));
        for (std::size_t i = 0; i < msg.colors.size(); ++i)
            toSoar(id, ("colors_" + std::to_string(i)).c_str(), msg.colors[i]);
        return id;
    }

    template <>
    inline visualization_msgs::msg::Marker
    fromSoar<visualization_msgs::msg::Marker>(sml::Identifier *id)
    {
        visualization_msgs::msg::Marker msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.ns = detail::getString(id, "ns");
        msg.id = static_cast<int32_t>(detail::getInt(id, "id"));
        msg.type = static_cast<int32_t>(detail::getInt(id, "type"));
        msg.action = static_cast<int32_t>(detail::getInt(id, "action"));
        auto *pose = detail::getChild(id, "pose");
        if (pose)
            msg.pose = fromSoar<geometry_msgs::msg::Pose>(pose);
        auto *scale = detail::getChild(id, "scale");
        if (scale)
            msg.scale = fromSoar<geometry_msgs::msg::Vector3>(scale);
        auto *color = detail::getChild(id, "color");
        if (color)
            msg.color = fromSoar<std_msgs::msg::ColorRGBA>(color);
        auto *lt = detail::getChild(id, "lifetime");
        if (lt)
            msg.lifetime = fromSoar<builtin_interfaces::msg::Duration>(lt);
        msg.frame_locked = detail::getInt(id, "frame_locked") != 0;
        msg.text = detail::getString(id, "text");
        msg.mesh_resource = detail::getString(id, "mesh_resource");
        int64_t np = detail::getInt(id, "points_count");
        for (int64_t i = 0; i < np; ++i)
        {
            auto *pt = detail::getChild(id, ("points_" + std::to_string(i)).c_str());
            if (pt)
                msg.points.push_back(fromSoar<geometry_msgs::msg::Point>(pt));
        }
        int64_t nc = detail::getInt(id, "colors_count");
        for (int64_t i = 0; i < nc; ++i)
        {
            auto *cl = detail::getChild(id, ("colors_" + std::to_string(i)).c_str());
            if (cl)
                msg.colors.push_back(fromSoar<std_msgs::msg::ColorRGBA>(cl));
        }
        return msg;
    }

    // ── MarkerArray ───────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- markers_count  (int)
    //     +-- markers_0, …   (ID)  → full Marker subtree

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const visualization_msgs::msg::MarkerArray &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("markers_count", static_cast<int64_t>(msg.markers.size()));
        for (std::size_t i = 0; i < msg.markers.size(); ++i)
            toSoar(id, ("markers_" + std::to_string(i)).c_str(), msg.markers[i]);
        return id;
    }

    template <>
    inline visualization_msgs::msg::MarkerArray
    fromSoar<visualization_msgs::msg::MarkerArray>(sml::Identifier *id)
    {
        visualization_msgs::msg::MarkerArray msg;
        int64_t count = detail::getInt(id, "markers_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("markers_" + std::to_string(i)).c_str());
            if (child)
                msg.markers.push_back(fromSoar<visualization_msgs::msg::Marker>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__VISUALIZATION_MSGS_CONVERTERS_HPP_
