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

#ifndef SOAR_ROS__MSG__CONVERTERS_HPP_
#define SOAR_ROS__MSG__CONVERTERS_HPP_

/// @file converters.hpp
/// @brief Umbrella header for ROS 2 ↔ Soar WME message converters.
///
/// Provides two free-function families in the @c soar_ros::msg namespace:
///
/// @par toSoar — ROS 2 message → Soar working memory
/// @code
///   sml::Identifier* toSoar(sml::Identifier* parent,
///                            const char*       attr,
///                            const MsgType&    msg);
/// @endcode
/// Creates a child ID WME under @p parent with attribute @p attr and
/// recursively populates it to mirror the ROS 2 message field hierarchy.
/// Returns the newly created @c sml::Identifier*.
///
/// Typical use inside a @c soar_ros::Subscriber<T>::parse() override:
/// @code
///   void parse(geometry_msgs::msg::PoseStamped msg) override
///   {
///     soar_ros::msg::toSoar(m_pAgent->GetInputLink(), m_topic.c_str(), msg);
///     // Resulting WME tree:
///     // <input-link>
///     //   +-- <topic> (ID)
///     //         +-- header (ID)
///     //         |     +-- frame_id  (string)
///     //         |     +-- stamp (ID)
///     //         |           +-- sec     (int)
///     //         |           +-- nanosec (int)
///     //         +-- pose (ID)
///     //               +-- position (ID)
///     //               |     +-- x (float)  y (float)  z (float)
///     //               +-- orientation (ID)
///     //                     +-- x (float)  y (float)  z (float)  w (float)
///   }
/// @endcode
///
/// @par fromSoar — Soar working memory → ROS 2 message
/// @code
///   T fromSoar<T>(sml::Identifier* id);
/// @endcode
/// Reads the WME subtree rooted at @p id and returns the corresponding
/// ROS 2 message of type @p T.
///
/// Typical use inside a @c soar_ros::Publisher<T>::parse() override:
/// @code
///   geometry_msgs::msg::Twist parse(sml::Identifier* id) override
///   {
///     // Soar rule writes:
///     //   (<output-link> ^cmd <c>)
///     //   (<c> ^linear <l> ^angular <a>)
///     //   (<l> ^x 1.0 ^y 0.0 ^z 0.0)
///     //   (<a> ^x 0.0 ^y 0.0 ^z 0.5)
///     return soar_ros::msg::fromSoar<geometry_msgs::msg::Twist>(id);
///   }
/// @endcode
///
/// @par Supported message types
///   - **std_msgs**  : Bool, Int8/16/32/64, UInt8/16/32/64, Float32/64,
///                     String, Header
///   - **geometry_msgs** : Vector3, Point, Point32, Quaternion,
///                         Pose, PoseStamped, PoseWithCovariance,
///                         Twist, TwistStamped, Accel,
///                         Transform, TransformStamped
///
/// @par WME attribute naming
/// Attribute names match the ROS 2 field names exactly, including underscores
/// (e.g. @c frame_id, @c child_frame_id, @c nanosec).  Nested message fields
/// become child ID WMEs.  The @c PoseWithCovariance covariance matrix is
/// stored as 36 flat float attributes named @c covariance_0 … @c covariance_35.

#include "soar_ros/msg/geometry_msgs_converters.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

#endif // SOAR_ROS__MSG__CONVERTERS_HPP_
