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

/// @file tf2_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for tf2_msgs message types.
///
/// Optional include — requires tf2_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__TF2_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__TF2_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <tf2_msgs/msg/tf_message.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/geometry_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── TFMessage ─────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- transforms_count  (int)
    //     +-- transforms_0, …   (ID)  → full TransformStamped subtree

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const tf2_msgs::msg::TFMessage &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("transforms_count", static_cast<int64_t>(msg.transforms.size()));
        for (std::size_t i = 0; i < msg.transforms.size(); ++i)
            toSoar(id, ("transforms_" + std::to_string(i)).c_str(), msg.transforms[i]);
        return id;
    }

    template <>
    inline tf2_msgs::msg::TFMessage
    fromSoar<tf2_msgs::msg::TFMessage>(sml::Identifier *id)
    {
        tf2_msgs::msg::TFMessage msg;
        int64_t count = detail::getInt(id, "transforms_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("transforms_" + std::to_string(i)).c_str());
            if (child)
                msg.transforms.push_back(
                    fromSoar<geometry_msgs::msg::TransformStamped>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__TF2_MSGS_CONVERTERS_HPP_
