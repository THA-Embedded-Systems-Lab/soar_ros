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

/// @file diagnostic_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for diagnostic_msgs message types.
///
/// Optional include — requires diagnostic_msgs in the consuming package.

#ifndef SOAR_ROS__MSG__DIAGNOSTIC_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__DIAGNOSTIC_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── KeyValue ──────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- key   (string)
    //     +-- value (string)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const diagnostic_msgs::msg::KeyValue &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateStringWME("key", msg.key.c_str());
        id->CreateStringWME("value", msg.value.c_str());
        return id;
    }

    template <>
    inline diagnostic_msgs::msg::KeyValue
    fromSoar<diagnostic_msgs::msg::KeyValue>(sml::Identifier *id)
    {
        diagnostic_msgs::msg::KeyValue msg;
        msg.key = detail::getString(id, "key");
        msg.value = detail::getString(id, "value");
        return msg;
    }

    // ── DiagnosticStatus ──────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- level        (int)    OK=0, WARN=1, ERROR=2, STALE=3
    //     +-- name         (string)
    //     +-- message      (string)
    //     +-- hardware_id  (string)
    //     +-- values_count (int)
    //     +-- values_0, …  (ID)    → key, value

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const diagnostic_msgs::msg::DiagnosticStatus &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("level", static_cast<int64_t>(msg.level));
        id->CreateStringWME("name", msg.name.c_str());
        id->CreateStringWME("message", msg.message.c_str());
        id->CreateStringWME("hardware_id", msg.hardware_id.c_str());
        id->CreateIntWME("values_count", static_cast<int64_t>(msg.values.size()));
        for (std::size_t i = 0; i < msg.values.size(); ++i)
            toSoar(id, ("values_" + std::to_string(i)).c_str(), msg.values[i]);
        return id;
    }

    template <>
    inline diagnostic_msgs::msg::DiagnosticStatus
    fromSoar<diagnostic_msgs::msg::DiagnosticStatus>(sml::Identifier *id)
    {
        diagnostic_msgs::msg::DiagnosticStatus msg;
        msg.level = static_cast<int8_t>(detail::getInt(id, "level"));
        msg.name = detail::getString(id, "name");
        msg.message = detail::getString(id, "message");
        msg.hardware_id = detail::getString(id, "hardware_id");
        int64_t count = detail::getInt(id, "values_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("values_" + std::to_string(i)).c_str());
            if (child)
                msg.values.push_back(fromSoar<diagnostic_msgs::msg::KeyValue>(child));
        }
        return msg;
    }

    // ── DiagnosticArray ───────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header        (ID)
    //     +-- status_count  (int)
    //     +-- status_0, …   (ID)  → DiagnosticStatus subtree

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const diagnostic_msgs::msg::DiagnosticArray &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateIntWME("status_count", static_cast<int64_t>(msg.status.size()));
        for (std::size_t i = 0; i < msg.status.size(); ++i)
            toSoar(id, ("status_" + std::to_string(i)).c_str(), msg.status[i]);
        return id;
    }

    template <>
    inline diagnostic_msgs::msg::DiagnosticArray
    fromSoar<diagnostic_msgs::msg::DiagnosticArray>(sml::Identifier *id)
    {
        diagnostic_msgs::msg::DiagnosticArray msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        int64_t count = detail::getInt(id, "status_count");
        for (int64_t i = 0; i < count; ++i)
        {
            auto *child = detail::getChild(id, ("status_" + std::to_string(i)).c_str());
            if (child)
                msg.status.push_back(fromSoar<diagnostic_msgs::msg::DiagnosticStatus>(child));
        }
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__DIAGNOSTIC_MSGS_CONVERTERS_HPP_
