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

#ifndef SOAR_ROS__MSG__STD_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__STD_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "soar_ros/msg/detail.hpp"

namespace soar_ros::msg
{

    // ── Bool ──────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- data (int)  1 = true, 0 = false

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Bool &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", msg.data ? 1 : 0);
        return id;
    }

    template <>
    inline std_msgs::msg::Bool fromSoar<std_msgs::msg::Bool>(sml::Identifier *id)
    {
        std_msgs::msg::Bool msg;
        msg.data = detail::getInt(id, "data") != 0;
        return msg;
    }

    // ── Int8 ──────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- data (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Int8 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::Int8 fromSoar<std_msgs::msg::Int8>(sml::Identifier *id)
    {
        std_msgs::msg::Int8 msg;
        msg.data = static_cast<int8_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── Int16 ─────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Int16 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::Int16 fromSoar<std_msgs::msg::Int16>(sml::Identifier *id)
    {
        std_msgs::msg::Int16 msg;
        msg.data = static_cast<int16_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── Int32 ─────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Int32 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::Int32 fromSoar<std_msgs::msg::Int32>(sml::Identifier *id)
    {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── Int64 ─────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Int64 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", msg.data);
        return id;
    }

    template <>
    inline std_msgs::msg::Int64 fromSoar<std_msgs::msg::Int64>(sml::Identifier *id)
    {
        std_msgs::msg::Int64 msg;
        msg.data = detail::getInt(id, "data");
        return msg;
    }

    // ── UInt8 ─────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::UInt8 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::UInt8 fromSoar<std_msgs::msg::UInt8>(sml::Identifier *id)
    {
        std_msgs::msg::UInt8 msg;
        msg.data = static_cast<uint8_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── UInt16 ────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::UInt16 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::UInt16 fromSoar<std_msgs::msg::UInt16>(sml::Identifier *id)
    {
        std_msgs::msg::UInt16 msg;
        msg.data = static_cast<uint16_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── UInt32 ────────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::UInt32 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::UInt32 fromSoar<std_msgs::msg::UInt32>(sml::Identifier *id)
    {
        std_msgs::msg::UInt32 msg;
        msg.data = static_cast<uint32_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── UInt64 ────────────────────────────────────────────────────────────────────
    // Note: Soar int is int64_t; UInt64 values greater than INT64_MAX will wrap.

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::UInt64 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("data", static_cast<int64_t>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::UInt64 fromSoar<std_msgs::msg::UInt64>(sml::Identifier *id)
    {
        std_msgs::msg::UInt64 msg;
        msg.data = static_cast<uint64_t>(detail::getInt(id, "data"));
        return msg;
    }

    // ── Float32 ───────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- data (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Float32 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("data", static_cast<double>(msg.data));
        return id;
    }

    template <>
    inline std_msgs::msg::Float32 fromSoar<std_msgs::msg::Float32>(sml::Identifier *id)
    {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(detail::getFloat(id, "data"));
        return msg;
    }

    // ── Float64 ───────────────────────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Float64 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("data", msg.data);
        return id;
    }

    template <>
    inline std_msgs::msg::Float64 fromSoar<std_msgs::msg::Float64>(sml::Identifier *id)
    {
        std_msgs::msg::Float64 msg;
        msg.data = detail::getFloat(id, "data");
        return msg;
    }

    // ── String ────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- data (string)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::String &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateStringWME("data", msg.data.c_str());
        return id;
    }

    template <>
    inline std_msgs::msg::String fromSoar<std_msgs::msg::String>(sml::Identifier *id)
    {
        std_msgs::msg::String msg;
        msg.data = detail::getString(id, "data");
        return msg;
    }

    // ── Header ────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- frame_id (string)
    //     +-- stamp (ID)
    //          +-- sec     (int)
    //          +-- nanosec (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::Header &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateStringWME("frame_id", msg.frame_id.c_str());
        auto *stamp = id->CreateIdWME("stamp");
        stamp->CreateIntWME("sec", static_cast<int64_t>(msg.stamp.sec));
        stamp->CreateIntWME("nanosec", static_cast<int64_t>(msg.stamp.nanosec));
        return id;
    }

    template <>
    inline std_msgs::msg::Header fromSoar<std_msgs::msg::Header>(sml::Identifier *id)
    {
        std_msgs::msg::Header msg;
        msg.frame_id = detail::getString(id, "frame_id");
        auto *stamp = detail::getChild(id, "stamp");
        if (stamp)
        {
            msg.stamp.sec = static_cast<int32_t>(detail::getInt(stamp, "sec"));
            msg.stamp.nanosec = static_cast<uint32_t>(detail::getInt(stamp, "nanosec"));
        }
        return msg;
    }

    // ── ColorRGBA ────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- r (float)
    //     +-- g (float)
    //     +-- b (float)
    //     +-- a (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const std_msgs::msg::ColorRGBA &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("r", static_cast<double>(msg.r));
        id->CreateFloatWME("g", static_cast<double>(msg.g));
        id->CreateFloatWME("b", static_cast<double>(msg.b));
        id->CreateFloatWME("a", static_cast<double>(msg.a));
        return id;
    }

    template <>
    inline std_msgs::msg::ColorRGBA fromSoar<std_msgs::msg::ColorRGBA>(sml::Identifier *id)
    {
        std_msgs::msg::ColorRGBA msg;
        msg.r = static_cast<float>(detail::getFloat(id, "r"));
        msg.g = static_cast<float>(detail::getFloat(id, "g"));
        msg.b = static_cast<float>(detail::getFloat(id, "b"));
        msg.a = static_cast<float>(detail::getFloat(id, "a"));
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__STD_MSGS_CONVERTERS_HPP_
