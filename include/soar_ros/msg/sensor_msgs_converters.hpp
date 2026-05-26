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

/// @file sensor_msgs_converters.hpp
/// @brief toSoar / fromSoar converters for sensor_msgs message types.
///
/// Optional include — requires sensor_msgs in the consuming package.
///
/// Variable-length arrays (ranges, intensities, joint positions, etc.) are
/// stored as indexed WMEs: a @c _count int WME plus @c field_0, @c field_1, …
/// See soar_ros/msg/detail.hpp for the helper functions.

#ifndef SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_
#define SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_

#include <sml_Client.h>

#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "soar_ros/msg/detail.hpp"
#include "soar_ros/msg/geometry_msgs_converters.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

namespace soar_ros::msg
{

    // ── Temperature ───────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header      (ID)  → frame_id, stamp
    //     +-- temperature (float)
    //     +-- variance    (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::Temperature &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateFloatWME("temperature", msg.temperature);
        id->CreateFloatWME("variance", msg.variance);
        return id;
    }

    template <>
    inline sensor_msgs::msg::Temperature
    fromSoar<sensor_msgs::msg::Temperature>(sml::Identifier *id)
    {
        sensor_msgs::msg::Temperature msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.temperature = detail::getFloat(id, "temperature");
        msg.variance = detail::getFloat(id, "variance");
        return msg;
    }

    // ── FluidPressure ─────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header         (ID)
    //     +-- fluid_pressure (float)
    //     +-- variance       (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::FluidPressure &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateFloatWME("fluid_pressure", msg.fluid_pressure);
        id->CreateFloatWME("variance", msg.variance);
        return id;
    }

    template <>
    inline sensor_msgs::msg::FluidPressure
    fromSoar<sensor_msgs::msg::FluidPressure>(sml::Identifier *id)
    {
        sensor_msgs::msg::FluidPressure msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.fluid_pressure = detail::getFloat(id, "fluid_pressure");
        msg.variance = detail::getFloat(id, "variance");
        return msg;
    }

    // ── Illuminance ───────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header     (ID)
    //     +-- illuminance (float)
    //     +-- variance   (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::Illuminance &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateFloatWME("illuminance", msg.illuminance);
        id->CreateFloatWME("variance", msg.variance);
        return id;
    }

    template <>
    inline sensor_msgs::msg::Illuminance
    fromSoar<sensor_msgs::msg::Illuminance>(sml::Identifier *id)
    {
        sensor_msgs::msg::Illuminance msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.illuminance = detail::getFloat(id, "illuminance");
        msg.variance = detail::getFloat(id, "variance");
        return msg;
    }

    // ── Range ─────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header         (ID)
    //     +-- radiation_type (int)   ULTRASOUND=0, INFRARED=1
    //     +-- field_of_view  (float)
    //     +-- min_range      (float)
    //     +-- max_range      (float)
    //     +-- range          (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::Range &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateIntWME("radiation_type", static_cast<int64_t>(msg.radiation_type));
        id->CreateFloatWME("field_of_view", static_cast<double>(msg.field_of_view));
        id->CreateFloatWME("min_range", static_cast<double>(msg.min_range));
        id->CreateFloatWME("max_range", static_cast<double>(msg.max_range));
        id->CreateFloatWME("range", static_cast<double>(msg.range));
        return id;
    }

    template <>
    inline sensor_msgs::msg::Range
    fromSoar<sensor_msgs::msg::Range>(sml::Identifier *id)
    {
        sensor_msgs::msg::Range msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.radiation_type = static_cast<uint8_t>(detail::getInt(id, "radiation_type"));
        msg.field_of_view = static_cast<float>(detail::getFloat(id, "field_of_view"));
        msg.min_range = static_cast<float>(detail::getFloat(id, "min_range"));
        msg.max_range = static_cast<float>(detail::getFloat(id, "max_range"));
        msg.range = static_cast<float>(detail::getFloat(id, "range"));
        return msg;
    }

    // ── NavSatStatus ──────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- status  (int)   STATUS_NO_FIX=-1, STATUS_FIX=0, ...
    //     +-- service (int)   bitmask

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::NavSatStatus &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        id->CreateIntWME("status", static_cast<int64_t>(msg.status));
        id->CreateIntWME("service", static_cast<int64_t>(msg.service));
        return id;
    }

    template <>
    inline sensor_msgs::msg::NavSatStatus
    fromSoar<sensor_msgs::msg::NavSatStatus>(sml::Identifier *id)
    {
        sensor_msgs::msg::NavSatStatus msg;
        msg.status = static_cast<int8_t>(detail::getInt(id, "status"));
        msg.service = static_cast<uint16_t>(detail::getInt(id, "service"));
        return msg;
    }

    // ── NavSatFix ─────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header                   (ID)
    //     +-- status                   (ID)  → status, service
    //     +-- latitude                 (float)
    //     +-- longitude                (float)
    //     +-- altitude                 (float)
    //     +-- position_covariance_0 … position_covariance_8  (float)
    //     +-- position_covariance_type (int)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::NavSatFix &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "status", msg.status);
        id->CreateFloatWME("latitude", msg.latitude);
        id->CreateFloatWME("longitude", msg.longitude);
        id->CreateFloatWME("altitude", msg.altitude);
        for (int i = 0; i < 9; ++i)
            id->CreateFloatWME(("position_covariance_" + std::to_string(i)).c_str(),
                               msg.position_covariance[i]);
        id->CreateIntWME("position_covariance_type",
                         static_cast<int64_t>(msg.position_covariance_type));
        return id;
    }

    template <>
    inline sensor_msgs::msg::NavSatFix
    fromSoar<sensor_msgs::msg::NavSatFix>(sml::Identifier *id)
    {
        sensor_msgs::msg::NavSatFix msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        auto *st = detail::getChild(id, "status");
        if (st)
            msg.status = fromSoar<sensor_msgs::msg::NavSatStatus>(st);
        msg.latitude = detail::getFloat(id, "latitude");
        msg.longitude = detail::getFloat(id, "longitude");
        msg.altitude = detail::getFloat(id, "altitude");
        for (int i = 0; i < 9; ++i)
            msg.position_covariance[i] =
                detail::getFloat(id, ("position_covariance_" + std::to_string(i)).c_str());
        msg.position_covariance_type =
            static_cast<uint8_t>(detail::getInt(id, "position_covariance_type"));
        return msg;
    }

    // ── MagneticField ─────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header                        (ID)
    //     +-- magnetic_field                (ID)  → x, y, z (float)
    //     +-- magnetic_field_covariance_0 … magnetic_field_covariance_8 (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::MagneticField &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "magnetic_field", msg.magnetic_field);
        for (int i = 0; i < 9; ++i)
            id->CreateFloatWME(("magnetic_field_covariance_" + std::to_string(i)).c_str(),
                               msg.magnetic_field_covariance[i]);
        return id;
    }

    template <>
    inline sensor_msgs::msg::MagneticField
    fromSoar<sensor_msgs::msg::MagneticField>(sml::Identifier *id)
    {
        sensor_msgs::msg::MagneticField msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        auto *mf = detail::getChild(id, "magnetic_field");
        if (mf)
            msg.magnetic_field = fromSoar<geometry_msgs::msg::Vector3>(mf);
        for (int i = 0; i < 9; ++i)
            msg.magnetic_field_covariance[i] =
                detail::getFloat(id, ("magnetic_field_covariance_" + std::to_string(i)).c_str());
        return msg;
    }

    // ── Imu ───────────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header                          (ID)
    //     +-- orientation                     (ID)  → x, y, z, w (float)
    //     +-- orientation_covariance_0 … _8   (float)
    //     +-- angular_velocity                (ID)  → x, y, z (float)
    //     +-- angular_velocity_covariance_0 … _8    (float)
    //     +-- linear_acceleration             (ID)  → x, y, z (float)
    //     +-- linear_acceleration_covariance_0 … _8 (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::Imu &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        toSoar(id, "orientation", msg.orientation);
        for (int i = 0; i < 9; ++i)
            id->CreateFloatWME(("orientation_covariance_" + std::to_string(i)).c_str(),
                               msg.orientation_covariance[i]);
        toSoar(id, "angular_velocity", msg.angular_velocity);
        for (int i = 0; i < 9; ++i)
            id->CreateFloatWME(("angular_velocity_covariance_" + std::to_string(i)).c_str(),
                               msg.angular_velocity_covariance[i]);
        toSoar(id, "linear_acceleration", msg.linear_acceleration);
        for (int i = 0; i < 9; ++i)
            id->CreateFloatWME(("linear_acceleration_covariance_" + std::to_string(i)).c_str(),
                               msg.linear_acceleration_covariance[i]);
        return id;
    }

    template <>
    inline sensor_msgs::msg::Imu
    fromSoar<sensor_msgs::msg::Imu>(sml::Identifier *id)
    {
        sensor_msgs::msg::Imu msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        auto *ori = detail::getChild(id, "orientation");
        if (ori)
            msg.orientation = fromSoar<geometry_msgs::msg::Quaternion>(ori);
        for (int i = 0; i < 9; ++i)
            msg.orientation_covariance[i] =
                detail::getFloat(id, ("orientation_covariance_" + std::to_string(i)).c_str());
        auto *av = detail::getChild(id, "angular_velocity");
        if (av)
            msg.angular_velocity = fromSoar<geometry_msgs::msg::Vector3>(av);
        for (int i = 0; i < 9; ++i)
            msg.angular_velocity_covariance[i] =
                detail::getFloat(id, ("angular_velocity_covariance_" + std::to_string(i)).c_str());
        auto *la = detail::getChild(id, "linear_acceleration");
        if (la)
            msg.linear_acceleration = fromSoar<geometry_msgs::msg::Vector3>(la);
        for (int i = 0; i < 9; ++i)
            msg.linear_acceleration_covariance[i] =
                detail::getFloat(id, ("linear_acceleration_covariance_" + std::to_string(i)).c_str());
        return msg;
    }

    // ── JointState ────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header             (ID)
    //     +-- name_count         (int)
    //     +-- name_0, name_1, …  (string)
    //     +-- position_count     (int)
    //     +-- position_0, …      (float)
    //     +-- velocity_count     (int)
    //     +-- velocity_0, …      (float)
    //     +-- effort_count       (int)
    //     +-- effort_0, …        (float)

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::JointState &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        detail::writeStringArray(id, "name", msg.name);
        detail::writeNumericArray(id, "position", msg.position);
        detail::writeNumericArray(id, "velocity", msg.velocity);
        detail::writeNumericArray(id, "effort", msg.effort);
        return id;
    }

    template <>
    inline sensor_msgs::msg::JointState
    fromSoar<sensor_msgs::msg::JointState>(sml::Identifier *id)
    {
        sensor_msgs::msg::JointState msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.name = detail::readStringArray(id, "name");
        msg.position = detail::readNumericArray<double>(id, "position");
        msg.velocity = detail::readNumericArray<double>(id, "velocity");
        msg.effort = detail::readNumericArray<double>(id, "effort");
        return msg;
    }

    // ── LaserScan ─────────────────────────────────────────────────────────────────
    // WME structure:
    //   <attr> (ID)
    //     +-- header            (ID)
    //     +-- angle_min         (float)
    //     +-- angle_max         (float)
    //     +-- angle_increment   (float)
    //     +-- time_increment    (float)
    //     +-- scan_time         (float)
    //     +-- range_min         (float)
    //     +-- range_max         (float)
    //     +-- ranges_count      (int)
    //     +-- ranges_0, …       (float)
    //     +-- intensities_count (int)
    //     +-- intensities_0, …  (float)
    //
    // Note: large scans (e.g. 1080 rays) produce many WMEs. Consider
    // downsampling before calling toSoar for performance-sensitive agents.

    inline sml::Identifier *toSoar(
        sml::Identifier *parent, const char *attr,
        const sensor_msgs::msg::LaserScan &msg)
    {
        auto *id = parent->CreateIdWME(attr);
        toSoar(id, "header", msg.header);
        id->CreateFloatWME("angle_min", static_cast<double>(msg.angle_min));
        id->CreateFloatWME("angle_max", static_cast<double>(msg.angle_max));
        id->CreateFloatWME("angle_increment", static_cast<double>(msg.angle_increment));
        id->CreateFloatWME("time_increment", static_cast<double>(msg.time_increment));
        id->CreateFloatWME("scan_time", static_cast<double>(msg.scan_time));
        id->CreateFloatWME("range_min", static_cast<double>(msg.range_min));
        id->CreateFloatWME("range_max", static_cast<double>(msg.range_max));
        detail::writeNumericArray(id, "ranges",
                                  std::vector<double>(msg.ranges.begin(), msg.ranges.end()));
        detail::writeNumericArray(id, "intensities",
                                  std::vector<double>(msg.intensities.begin(), msg.intensities.end()));
        return id;
    }

    template <>
    inline sensor_msgs::msg::LaserScan
    fromSoar<sensor_msgs::msg::LaserScan>(sml::Identifier *id)
    {
        sensor_msgs::msg::LaserScan msg;
        auto *hdr = detail::getChild(id, "header");
        if (hdr)
            msg.header = fromSoar<std_msgs::msg::Header>(hdr);
        msg.angle_min = static_cast<float>(detail::getFloat(id, "angle_min"));
        msg.angle_max = static_cast<float>(detail::getFloat(id, "angle_max"));
        msg.angle_increment = static_cast<float>(detail::getFloat(id, "angle_increment"));
        msg.time_increment = static_cast<float>(detail::getFloat(id, "time_increment"));
        msg.scan_time = static_cast<float>(detail::getFloat(id, "scan_time"));
        msg.range_min = static_cast<float>(detail::getFloat(id, "range_min"));
        msg.range_max = static_cast<float>(detail::getFloat(id, "range_max"));
        auto r = detail::readNumericArray<double>(id, "ranges");
        msg.ranges.assign(r.begin(), r.end());
        auto inten = detail::readNumericArray<double>(id, "intensities");
        msg.intensities.assign(inten.begin(), inten.end());
        return msg;
    }

} // namespace soar_ros::msg

#endif // SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_
