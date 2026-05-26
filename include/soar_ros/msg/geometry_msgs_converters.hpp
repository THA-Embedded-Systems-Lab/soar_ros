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

#ifndef SOAR_ROS__MSG__GEOMETRY_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__GEOMETRY_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── Vector3 ───────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- x (float)
    //     +-- y (float)
    //     +-- z (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Vector3 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("x", msg.x);
        id->CreateFloatWME("y", msg.y);
        id->CreateFloatWME("z", msg.z);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Vector3
    fromSoar<geometry_msgs::msg::Vector3>(sml::Identifier *id)
    {
        geometry_msgs::msg::Vector3 msg;
        msg.x = detail::getFloat(id, "x");
        msg.y = detail::getFloat(id, "y");
        msg.z = detail::getFloat(id, "z");
        return msg;
    }

    // ── Point ─────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- x (float)
    //     +-- y (float)
    //     +-- z (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Point &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("x", msg.x);
        id->CreateFloatWME("y", msg.y);
        id->CreateFloatWME("z", msg.z);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Point
    fromSoar<geometry_msgs::msg::Point>(sml::Identifier *id)
    {
        geometry_msgs::msg::Point msg;
        msg.x = detail::getFloat(id, "x");
        msg.y = detail::getFloat(id, "y");
        msg.z = detail::getFloat(id, "z");
        return msg;
    }

    // ── Point32 ───────────────────────────────────────────────────────────────────
    // Note: float32 fields are widened to Soar float (double) on conversion.

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Point32 &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("x", static_cast<double>(msg.x));
        id->CreateFloatWME("y", static_cast<double>(msg.y));
        id->CreateFloatWME("z", static_cast<double>(msg.z));
        return id;
    }

    template <>
    inline geometry_msgs::msg::Point32
    fromSoar<geometry_msgs::msg::Point32>(sml::Identifier *id)
    {
        geometry_msgs::msg::Point32 msg;
        msg.x = static_cast<float>(detail::getFloat(id, "x"));
        msg.y = static_cast<float>(detail::getFloat(id, "y"));
        msg.z = static_cast<float>(detail::getFloat(id, "z"));
        return msg;
    }

    // ── Quaternion ────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- x (float)
    //     +-- y (float)
    //     +-- z (float)
    //     +-- w (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Quaternion &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateFloatWME("x", msg.x);
        id->CreateFloatWME("y", msg.y);
        id->CreateFloatWME("z", msg.z);
        id->CreateFloatWME("w", msg.w);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Quaternion
    fromSoar<geometry_msgs::msg::Quaternion>(sml::Identifier *id)
    {
        geometry_msgs::msg::Quaternion msg;
        msg.x = detail::getFloat(id, "x");
        msg.y = detail::getFloat(id, "y");
        msg.z = detail::getFloat(id, "z");
        msg.w = detail::getFloat(id, "w");
        return msg;
    }

    // ── Pose ──────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- position    (ID) → Point
    //     +-- orientation (ID) → Quaternion

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Pose &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "position", msg.position);
        toSoar(id, "orientation", msg.orientation);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Pose
    fromSoar<geometry_msgs::msg::Pose>(sml::Identifier *id)
    {
        geometry_msgs::msg::Pose msg;
        auto *pos = detail::getChild(id, "position");
        if (pos)
        {
            msg.position = fromSoar<geometry_msgs::msg::Point>(pos);
        }
        auto *ori = detail::getChild(id, "orientation");
        if (ori)
        {
            msg.orientation = fromSoar<geometry_msgs::msg::Quaternion>(ori);
        }
        return msg;
    }

    // ── PoseStamped ───────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header (ID) → Header
    //     +-- pose   (ID) → Pose

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::PoseStamped &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "pose", msg.pose);
        return id;
    }

    template <>
    inline geometry_msgs::msg::PoseStamped
    fromSoar<geometry_msgs::msg::PoseStamped>(sml::Identifier *id)
    {
        geometry_msgs::msg::PoseStamped msg;
        auto *header = detail::getChild(id, "header");
        if (header)
        {
            msg.header = fromSoar<std_msgs::msg::Header>(header);
        }
        auto *pose = detail::getChild(id, "pose");
        if (pose)
        {
            msg.pose = fromSoar<geometry_msgs::msg::Pose>(pose);
        }
        return msg;
    }

    // ── PoseWithCovariance ────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- pose           (ID)    → Pose
    //     +-- covariance_0   (float)
    //     +-- covariance_1   (float)
    //     ...
    //     +-- covariance_35  (float)
    // The covariance is stored as a flat row-major 6×6 matrix with indexed attributes.

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::PoseWithCovariance &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "pose", msg.pose);
        for (std::size_t i = 0; i < msg.covariance.size(); ++i)
        {
            std::string key = "covariance_" + std::to_string(i);
            id->CreateFloatWME(key.c_str(), msg.covariance[i]);
        }
        return id;
    }

    template <>
    inline geometry_msgs::msg::PoseWithCovariance
    fromSoar<geometry_msgs::msg::PoseWithCovariance>(sml::Identifier *id)
    {
        geometry_msgs::msg::PoseWithCovariance msg;
        auto *pose = detail::getChild(id, "pose");
        if (pose)
        {
            msg.pose = fromSoar<geometry_msgs::msg::Pose>(pose);
        }
        for (std::size_t i = 0; i < msg.covariance.size(); ++i)
        {
            std::string key = "covariance_" + std::to_string(i);
            msg.covariance[i] = detail::getFloat(id, key.c_str());
        }
        return msg;
    }

    // ── Twist ─────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- linear  (ID) → Vector3
    //     +-- angular (ID) → Vector3

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Twist &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "linear", msg.linear);
        toSoar(id, "angular", msg.angular);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Twist
    fromSoar<geometry_msgs::msg::Twist>(sml::Identifier *id)
    {
        geometry_msgs::msg::Twist msg;
        auto *lin = detail::getChild(id, "linear");
        if (lin)
        {
            msg.linear = fromSoar<geometry_msgs::msg::Vector3>(lin);
        }
        auto *ang = detail::getChild(id, "angular");
        if (ang)
        {
            msg.angular = fromSoar<geometry_msgs::msg::Vector3>(ang);
        }
        return msg;
    }

    // ── TwistStamped ──────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header (ID) → Header
    //     +-- twist  (ID) → Twist

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::TwistStamped &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "twist", msg.twist);
        return id;
    }

    template <>
    inline geometry_msgs::msg::TwistStamped
    fromSoar<geometry_msgs::msg::TwistStamped>(sml::Identifier *id)
    {
        geometry_msgs::msg::TwistStamped msg;
        auto *header = detail::getChild(id, "header");
        if (header)
        {
            msg.header = fromSoar<std_msgs::msg::Header>(header);
        }
        auto *twist = detail::getChild(id, "twist");
        if (twist)
        {
            msg.twist = fromSoar<geometry_msgs::msg::Twist>(twist);
        }
        return msg;
    }

    // ── Accel ─────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- linear  (ID) → Vector3
    //     +-- angular (ID) → Vector3

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Accel &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "linear", msg.linear);
        toSoar(id, "angular", msg.angular);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Accel
    fromSoar<geometry_msgs::msg::Accel>(sml::Identifier *id)
    {
        geometry_msgs::msg::Accel msg;
        auto *lin = detail::getChild(id, "linear");
        if (lin)
        {
            msg.linear = fromSoar<geometry_msgs::msg::Vector3>(lin);
        }
        auto *ang = detail::getChild(id, "angular");
        if (ang)
        {
            msg.angular = fromSoar<geometry_msgs::msg::Vector3>(ang);
        }
        return msg;
    }

    // ── Transform ─────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- translation (ID) → Vector3
    //     +-- rotation    (ID) → Quaternion

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::Transform &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "translation", msg.translation);
        toSoar(id, "rotation", msg.rotation);
        return id;
    }

    template <>
    inline geometry_msgs::msg::Transform
    fromSoar<geometry_msgs::msg::Transform>(sml::Identifier *id)
    {
        geometry_msgs::msg::Transform msg;
        auto *trans = detail::getChild(id, "translation");
        if (trans)
        {
            msg.translation = fromSoar<geometry_msgs::msg::Vector3>(trans);
        }
        auto *rot = detail::getChild(id, "rotation");
        if (rot)
        {
            msg.rotation = fromSoar<geometry_msgs::msg::Quaternion>(rot);
        }
        return msg;
    }

    // ── TransformStamped ──────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header         (ID)     → Header
    //     +-- child_frame_id (string)
    //     +-- transform      (ID)     → Transform

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::TransformStamped &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateStringWME("child_frame_id", msg.child_frame_id.c_str());
        toSoar(id, "transform", msg.transform);
        return id;
    }

    template <>
    inline geometry_msgs::msg::TransformStamped
    fromSoar<geometry_msgs::msg::TransformStamped>(sml::Identifier *id)
    {
        geometry_msgs::msg::TransformStamped msg;
        auto *header = detail::getChild(id, "header");
        if (header)
        {
            msg.header = fromSoar<std_msgs::msg::Header>(header);
        }
        msg.child_frame_id = detail::getString(id, "child_frame_id");
        auto *tf = detail::getChild(id, "transform");
        if (tf)
        {
            msg.transform = fromSoar<geometry_msgs::msg::Transform>(tf);
        }
        return msg;
    }

    // ── TwistWithCovariance ───────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- twist (ID)  →  linear (ID) →  x, y, z (float)
    //     |                  angular (ID) →  x, y, z (float)
    //     +-- covariance_0 … covariance_35 (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::TwistWithCovariance &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "twist", msg.twist);
        for (int i = 0; i < 36; ++i)
            id->CreateFloatWME(("covariance_" + std::to_string(i)).c_str(), msg.covariance[i]);
        return id;
    }

    template <>
    inline geometry_msgs::msg::TwistWithCovariance
    fromSoar<geometry_msgs::msg::TwistWithCovariance>(sml::Identifier *id)
    {
        geometry_msgs::msg::TwistWithCovariance msg;
        auto *tw = detail::getChild(id, "twist");
        if (tw)
            msg.twist = fromSoar<geometry_msgs::msg::Twist>(tw);
        for (int i = 0; i < 36; ++i)
            msg.covariance[i] = detail::getFloat(id, ("covariance_" + std::to_string(i)).c_str());
        return msg;
    }

    // ── TwistWithCovarianceStamped ────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header (ID)
    //     +-- twist  (ID)  →  twist, covariance_0 … covariance_35

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::TwistWithCovarianceStamped &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "twist", msg.twist);
        return id;
    }

    template <>
    inline geometry_msgs::msg::TwistWithCovarianceStamped
    fromSoar<geometry_msgs::msg::TwistWithCovarianceStamped>(sml::Identifier *id)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        auto *tw = detail::getChild(id, "twist");
        if (tw)
            msg.twist = fromSoar<geometry_msgs::msg::TwistWithCovariance>(tw);
        return msg;
    }

    // ── AccelWithCovariance ───────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- accel (ID)  →  linear (ID) →  x, y, z (float)
    //     |                  angular (ID) →  x, y, z (float)
    //     +-- covariance_0 … covariance_35 (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::AccelWithCovariance &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "accel", msg.accel);
        for (int i = 0; i < 36; ++i)
            id->CreateFloatWME(("covariance_" + std::to_string(i)).c_str(), msg.covariance[i]);
        return id;
    }

    template <>
    inline geometry_msgs::msg::AccelWithCovariance
    fromSoar<geometry_msgs::msg::AccelWithCovariance>(sml::Identifier *id)
    {
        geometry_msgs::msg::AccelWithCovariance msg;
        auto *ac = detail::getChild(id, "accel");
        if (ac)
            msg.accel = fromSoar<geometry_msgs::msg::Accel>(ac);
        for (int i = 0; i < 36; ++i)
            msg.covariance[i] = detail::getFloat(id, ("covariance_" + std::to_string(i)).c_str());
        return msg;
    }

    // ── AccelWithCovarianceStamped ────────────────────────────────────────────────

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const geometry_msgs::msg::AccelWithCovarianceStamped &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "accel", msg.accel);
        return id;
    }

    template <>
    inline geometry_msgs::msg::AccelWithCovarianceStamped
    fromSoar<geometry_msgs::msg::AccelWithCovarianceStamped>(sml::Identifier *id)
    {
        geometry_msgs::msg::AccelWithCovarianceStamped msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        auto *ac = detail::getChild(id, "accel");
        if (ac)
            msg.accel = fromSoar<geometry_msgs::msg::AccelWithCovariance>(ac);
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__GEOMETRY_MSGS_CONVERTERS_HPP_
