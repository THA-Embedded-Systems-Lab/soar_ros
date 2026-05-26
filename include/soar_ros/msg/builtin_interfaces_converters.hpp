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

/// @file builtin_interfaces_converters.hpp
/// @brief toSoar / fromSoar converters for builtin_interfaces message types.
///
/// Optional include — requires builtin_interfaces in the consuming package.
/// Include this header (or soar_ros/soar_ros.hpp) before calling the converters.

#ifndef SOAR_ROS__MSG__BUILTIN_INTERFACES_CONVERTERS_HPP_
#define SOAR_ROS__MSG__BUILTIN_INTERFACES_CONVERTERS_HPP_

#include <sml_Client.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "soar_ros/msg/detail.hpp"

namespace soar_ros::msg
{

    // ── Time ──────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- sec     (int)
    //     +-- nanosec (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const builtin_interfaces::msg::Time &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("sec", static_cast<int64_t>(msg.sec));
        id->CreateIntWME("nanosec", static_cast<int64_t>(msg.nanosec));
        return id;
    }

    template <>
    inline builtin_interfaces::msg::Time
    fromSoar<builtin_interfaces::msg::Time>(sml::Identifier *id)
    {
        builtin_interfaces::msg::Time msg;
        msg.sec = static_cast<int32_t>(detail::getInt(id, "sec"));
        msg.nanosec = static_cast<uint32_t>(detail::getInt(id, "nanosec"));
        return msg;
    }

    // ── Duration ──────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- sec     (int)
    //     +-- nanosec (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const builtin_interfaces::msg::Duration &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("sec", static_cast<int64_t>(msg.sec));
        id->CreateIntWME("nanosec", static_cast<int64_t>(msg.nanosec));
        return id;
    }

    template <>
    inline builtin_interfaces::msg::Duration
    fromSoar<builtin_interfaces::msg::Duration>(sml::Identifier *id)
    {
        builtin_interfaces::msg::Duration msg;
        msg.sec = static_cast<int32_t>(detail::getInt(id, "sec"));
        msg.nanosec = static_cast<int32_t>(detail::getInt(id, "nanosec"));
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__BUILTIN_INTERFACES_CONVERTERS_HPP_
