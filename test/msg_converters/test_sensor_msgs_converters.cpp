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

#include <gtest/gtest.h>
#include <sml_Client.h>

#include "soar_ros/msg/sensor_msgs_converters.hpp"

class SensorMsgsConverterTest : public ::testing::Test
{
protected:
    sml::Kernel *kernel{nullptr};
    sml::Agent *agent{nullptr};

    void SetUp() override
    {
        kernel = sml::Kernel::CreateKernelInCurrentThread();
        ASSERT_NE(kernel, nullptr);
        agent = kernel->CreateAgent("test_agent");
        ASSERT_NE(agent, nullptr);
    }

    void TearDown() override
    {
        if (kernel)
        {
            kernel->Shutdown();
            delete kernel;
        }
    }

    sml::Identifier *inputLink() { return agent->GetInputLink(); }
};

// ── Temperature ───────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, Temperature_toSoar_Attributes)
{
    sensor_msgs::msg::Temperature msg;
    msg.temperature = 25.5;
    msg.variance = 0.1;
    auto *id = soar_ros::msg::toSoar(inputLink(), "temp", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("header", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("temperature", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("variance", 0), nullptr);
}

TEST_F(SensorMsgsConverterTest, Temperature_RoundTrip)
{
    sensor_msgs::msg::Temperature in;
    in.header.frame_id = "base_link";
    in.temperature = 36.6;
    in.variance = 0.01;
    auto *id = soar_ros::msg::toSoar(inputLink(), "temp", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::Temperature>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.temperature, in.temperature);
    EXPECT_DOUBLE_EQ(out.variance, in.variance);
}

// ── FluidPressure ─────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, FluidPressure_RoundTrip)
{
    sensor_msgs::msg::FluidPressure in;
    in.header.frame_id = "sensor";
    in.fluid_pressure = 101325.0;
    in.variance = 5.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "fp", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::FluidPressure>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.fluid_pressure, in.fluid_pressure);
    EXPECT_DOUBLE_EQ(out.variance, in.variance);
}

// ── Illuminance ───────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, Illuminance_RoundTrip)
{
    sensor_msgs::msg::Illuminance in;
    in.illuminance = 500.0;
    in.variance = 2.5;
    auto *id = soar_ros::msg::toSoar(inputLink(), "il", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::Illuminance>(id);
    EXPECT_DOUBLE_EQ(out.illuminance, in.illuminance);
    EXPECT_DOUBLE_EQ(out.variance, in.variance);
}

// ── Range ─────────────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, Range_toSoar_Attributes)
{
    sensor_msgs::msg::Range msg;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.range = 1.5f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "r", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("radiation_type"), "0");
    ASSERT_NE(id->FindByAttribute("range", 0), nullptr);
}

TEST_F(SensorMsgsConverterTest, Range_RoundTrip)
{
    sensor_msgs::msg::Range in;
    in.header.frame_id = "sonar";
    in.radiation_type = sensor_msgs::msg::Range::INFRARED;
    in.field_of_view = 0.5f;
    in.min_range = 0.05f;
    in.max_range = 5.0f;
    in.range = 2.3f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "r", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::Range>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_EQ(out.radiation_type, in.radiation_type);
    EXPECT_FLOAT_EQ(out.field_of_view, in.field_of_view);
    EXPECT_FLOAT_EQ(out.min_range, in.min_range);
    EXPECT_FLOAT_EQ(out.max_range, in.max_range);
    EXPECT_FLOAT_EQ(out.range, in.range);
}

// ── NavSatStatus ──────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, NavSatStatus_RoundTrip)
{
    sensor_msgs::msg::NavSatStatus in;
    in.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    in.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    auto *id = soar_ros::msg::toSoar(inputLink(), "ns", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::NavSatStatus>(id);
    EXPECT_EQ(out.status, in.status);
    EXPECT_EQ(out.service, in.service);
}

// ── NavSatFix ─────────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, NavSatFix_toSoar_Attributes)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = 48.137;
    msg.longitude = 11.575;
    msg.altitude = 520.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "fix", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("latitude", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("longitude", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("altitude", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("position_covariance_0", 0), nullptr);
}

TEST_F(SensorMsgsConverterTest, NavSatFix_RoundTrip)
{
    sensor_msgs::msg::NavSatFix in;
    in.header.frame_id = "gps";
    in.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    in.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    in.latitude = 48.137154;
    in.longitude = 11.574918;
    in.altitude = 519.5;
    in.position_covariance[0] = 0.1;
    in.position_covariance[4] = 0.1;
    in.position_covariance[8] = 0.25;
    in.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    auto *id = soar_ros::msg::toSoar(inputLink(), "fix", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::NavSatFix>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.latitude, in.latitude);
    EXPECT_DOUBLE_EQ(out.longitude, in.longitude);
    EXPECT_DOUBLE_EQ(out.altitude, in.altitude);
    EXPECT_DOUBLE_EQ(out.position_covariance[0], in.position_covariance[0]);
    EXPECT_DOUBLE_EQ(out.position_covariance[8], in.position_covariance[8]);
    EXPECT_EQ(out.position_covariance_type, in.position_covariance_type);
}

// ── MagneticField ─────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, MagneticField_toSoar_Attributes)
{
    sensor_msgs::msg::MagneticField msg;
    msg.magnetic_field.x = 0.00002;
    msg.magnetic_field.y = 0.00003;
    msg.magnetic_field.z = 0.00005;
    auto *id = soar_ros::msg::toSoar(inputLink(), "mf", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "magnetic_field"), nullptr);
    ASSERT_NE(id->FindByAttribute("magnetic_field_covariance_0", 0), nullptr);
}

TEST_F(SensorMsgsConverterTest, MagneticField_RoundTrip)
{
    sensor_msgs::msg::MagneticField in;
    in.header.frame_id = "imu_link";
    in.magnetic_field.x = 0.00002;
    in.magnetic_field.y = -0.00003;
    in.magnetic_field.z = 0.00005;
    in.magnetic_field_covariance[0] = 1e-6;
    in.magnetic_field_covariance[4] = 1e-6;
    in.magnetic_field_covariance[8] = 1e-6;
    auto *id = soar_ros::msg::toSoar(inputLink(), "mf", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::MagneticField>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.magnetic_field.x, in.magnetic_field.x);
    EXPECT_DOUBLE_EQ(out.magnetic_field.y, in.magnetic_field.y);
    EXPECT_DOUBLE_EQ(out.magnetic_field.z, in.magnetic_field.z);
    EXPECT_DOUBLE_EQ(out.magnetic_field_covariance[4], in.magnetic_field_covariance[4]);
}

// ── Imu ───────────────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, Imu_toSoar_Attributes)
{
    sensor_msgs::msg::Imu msg;
    msg.orientation.w = 1.0;
    msg.angular_velocity.z = 0.1;
    msg.linear_acceleration.z = -9.81;
    auto *id = soar_ros::msg::toSoar(inputLink(), "imu", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "orientation"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "angular_velocity"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "linear_acceleration"), nullptr);
    ASSERT_NE(id->FindByAttribute("orientation_covariance_0", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("linear_acceleration_covariance_8", 0), nullptr);
}

TEST_F(SensorMsgsConverterTest, Imu_RoundTrip)
{
    sensor_msgs::msg::Imu in;
    in.header.frame_id = "imu";
    in.orientation.x = 0.0;
    in.orientation.y = 0.0;
    in.orientation.z = 0.0;
    in.orientation.w = 1.0;
    in.angular_velocity.x = 0.01;
    in.angular_velocity.y = -0.02;
    in.angular_velocity.z = 0.1;
    in.linear_acceleration.x = 0.1;
    in.linear_acceleration.y = -0.05;
    in.linear_acceleration.z = -9.81;
    in.orientation_covariance[0] = 0.001;
    in.linear_acceleration_covariance[8] = 0.01;
    auto *id = soar_ros::msg::toSoar(inputLink(), "imu", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::Imu>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.orientation.w, in.orientation.w);
    EXPECT_DOUBLE_EQ(out.angular_velocity.z, in.angular_velocity.z);
    EXPECT_DOUBLE_EQ(out.linear_acceleration.z, in.linear_acceleration.z);
    EXPECT_DOUBLE_EQ(out.orientation_covariance[0], in.orientation_covariance[0]);
    EXPECT_DOUBLE_EQ(out.linear_acceleration_covariance[8], in.linear_acceleration_covariance[8]);
}

// ── JointState ────────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, JointState_toSoar_ArrayAttributes)
{
    sensor_msgs::msg::JointState msg;
    msg.name = {"joint1", "joint2", "joint3"};
    msg.position = {0.1, -0.2, 1.57};
    msg.velocity = {0.0, 0.0, 0.0};
    auto *id = soar_ros::msg::toSoar(inputLink(), "js", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("name_count"), "3");
    EXPECT_STREQ(id->GetParameterValue("name_0"), "joint1");
    EXPECT_STREQ(id->GetParameterValue("name_2"), "joint3");
    EXPECT_STREQ(id->GetParameterValue("position_count"), "3");
}

TEST_F(SensorMsgsConverterTest, JointState_RoundTrip)
{
    sensor_msgs::msg::JointState in;
    in.header.frame_id = "robot";
    in.name = {"shoulder", "elbow", "wrist"};
    in.position = {0.5, -1.0, 0.785};
    in.velocity = {0.01, -0.02, 0.005};
    in.effort = {1.5, 2.0, 0.8};
    auto *id = soar_ros::msg::toSoar(inputLink(), "js", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::JointState>(id);
    ASSERT_EQ(out.name.size(), in.name.size());
    EXPECT_EQ(out.name[0], in.name[0]);
    EXPECT_EQ(out.name[2], in.name[2]);
    ASSERT_EQ(out.position.size(), in.position.size());
    EXPECT_DOUBLE_EQ(out.position[0], in.position[0]);
    EXPECT_DOUBLE_EQ(out.position[2], in.position[2]);
    ASSERT_EQ(out.effort.size(), in.effort.size());
    EXPECT_DOUBLE_EQ(out.effort[1], in.effort[1]);
}

// ── LaserScan ─────────────────────────────────────────────────────────────────

TEST_F(SensorMsgsConverterTest, LaserScan_toSoar_Attributes)
{
    sensor_msgs::msg::LaserScan msg;
    msg.angle_min = -1.57f;
    msg.angle_max = 1.57f;
    msg.angle_increment = 0.01f;
    msg.range_min = 0.1f;
    msg.range_max = 10.0f;
    msg.ranges = {1.0f, 2.0f, 3.0f};
    auto *id = soar_ros::msg::toSoar(inputLink(), "ls", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("angle_min", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("angle_max", 0), nullptr);
    EXPECT_STREQ(id->GetParameterValue("ranges_count"), "3");
}

TEST_F(SensorMsgsConverterTest, LaserScan_RoundTrip)
{
    sensor_msgs::msg::LaserScan in;
    in.header.frame_id = "laser";
    in.angle_min = -1.5707963f;
    in.angle_max = 1.5707963f;
    in.angle_increment = 0.0174533f;
    in.time_increment = 0.0f;
    in.scan_time = 0.1f;
    in.range_min = 0.1f;
    in.range_max = 10.0f;
    in.ranges = {1.2f, 2.5f, 0.8f, 5.0f};
    in.intensities = {100.0f, 200.0f, 150.0f, 50.0f};
    auto *id = soar_ros::msg::toSoar(inputLink(), "ls", in);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::LaserScan>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_FLOAT_EQ(out.angle_min, in.angle_min);
    EXPECT_FLOAT_EQ(out.angle_max, in.angle_max);
    EXPECT_FLOAT_EQ(out.range_min, in.range_min);
    ASSERT_EQ(out.ranges.size(), in.ranges.size());
    EXPECT_FLOAT_EQ(out.ranges[0], in.ranges[0]);
    EXPECT_FLOAT_EQ(out.ranges[3], in.ranges[3]);
    ASSERT_EQ(out.intensities.size(), in.intensities.size());
    EXPECT_FLOAT_EQ(out.intensities[1], in.intensities[1]);
}
