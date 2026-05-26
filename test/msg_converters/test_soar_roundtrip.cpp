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

/// @file test_soar_roundtrip.cpp
/// @brief End-to-end round-trip tests that write a ROS message to the Soar
///        input-link, run the agent, read the value back from the output-link,
///        and verify the message contents are preserved.
///
/// The Soar agent uses a single copy rule:
///   1. Propose an operator that notes the `cmd` identifier on the input-link.
///   2. Apply the operator by writing that identifier to `output-link ^cmd`.
/// After RunSelf(3) the output-link carries the same identifier subtree that
/// was written to the input-link, so fromSoar sees identical WME values.

#include <gtest/gtest.h>
#include <sml_Client.h>

#include "soar_ros/msg/geometry_msgs_converters.hpp"
#include "soar_ros/msg/nav_msgs_converters.hpp"
#include "soar_ros/msg/sensor_msgs_converters.hpp"
#include "soar_ros/msg/std_msgs_converters.hpp"

// ── Fixture ───────────────────────────────────────────────────────────────────

class SoarRoundTripTest : public ::testing::Test
{
protected:
    sml::Kernel *kernel{nullptr};
    sml::Agent *agent{nullptr};

    void SetUp() override
    {
        kernel = sml::Kernel::CreateKernelInCurrentThread();
        ASSERT_NE(kernel, nullptr);
        agent = kernel->CreateAgent("rt_agent");
        ASSERT_NE(agent, nullptr);

        // Rule 1: propose a "copy" operator whenever there is a ^cmd on the
        //         input-link (= a message has been written by toSoar).
        agent->ExecuteCommandLine(
            "sp {roundtrip*copy*propose\n"
            "   (state <s> ^io.input-link.cmd <input>)\n"
            "-->\n"
            "   (<s> ^operator <o> + =)\n"
            "   (<o> ^name copy ^input <input>)\n"
            "}");

        // Rule 2: apply the operator by linking the input identifier to the
        //         output-link under the same ^cmd attribute.
        agent->ExecuteCommandLine(
            "sp {roundtrip*copy*apply\n"
            "   (state <s> ^operator <o> ^io.output-link <ol>)\n"
            "   (<o> ^name copy ^input <input>)\n"
            "-->\n"
            "   (<ol> ^cmd <input>)\n"
            "}");
    }

    void TearDown() override
    {
        if (kernel)
        {
            kernel->Shutdown();
            delete kernel;
            kernel = nullptr;
            agent = nullptr;
        }
    }

    /// Write a message to the input-link, run 3 decision cycles (init-agent
    /// has already fired at CreateAgent time, so cycle 1 = propose, cycle 2 =
    /// apply, cycle 3 = output phase committed), then return the identifier
    /// hanging off output-link ^cmd.
    template <typename Msg>
    sml::Identifier *runThrough(const Msg &msg)
    {
        soar_ros::msg::toSoar(agent->GetInputLink(), "cmd", msg);
        agent->RunSelf(3);
        auto *ol = agent->GetOutputLink();
        if (!ol)
        {
            return nullptr;
        }
        auto *wme = ol->FindByAttribute("cmd", 0);
        if (!wme)
        {
            return nullptr;
        }
        return wme->ConvertToIdentifier();
    }
};

// ── std_msgs::msg::String ─────────────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_String)
{
    std_msgs::msg::String in;
    in.data = "hello soar";
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<std_msgs::msg::String>(out_id);
    EXPECT_EQ(out.data, in.data);
}

// ── std_msgs::msg::Header ─────────────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_Header)
{
    std_msgs::msg::Header in;
    in.stamp.sec = 1748260000;
    in.stamp.nanosec = 123456789;
    in.frame_id = "map";
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<std_msgs::msg::Header>(out_id);
    EXPECT_EQ(out.frame_id, in.frame_id);
    EXPECT_EQ(out.stamp.sec, in.stamp.sec);
    EXPECT_EQ(out.stamp.nanosec, in.stamp.nanosec);
}

// ── geometry_msgs::msg::Twist ─────────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_Twist)
{
    geometry_msgs::msg::Twist in;
    in.linear.x = 1.5;
    in.linear.y = 0.0;
    in.linear.z = -0.3;
    in.angular.x = 0.0;
    in.angular.y = 0.0;
    in.angular.z = 0.785;
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<geometry_msgs::msg::Twist>(out_id);
    EXPECT_DOUBLE_EQ(out.linear.x, in.linear.x);
    EXPECT_DOUBLE_EQ(out.linear.z, in.linear.z);
    EXPECT_DOUBLE_EQ(out.angular.z, in.angular.z);
}

// ── geometry_msgs::msg::PoseStamped ──────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_PoseStamped)
{
    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = "world";
    in.header.stamp.sec = 42;
    in.pose.position.x = 3.14;
    in.pose.position.y = -2.71;
    in.pose.position.z = 0.5;
    in.pose.orientation.x = 0.0;
    in.pose.orientation.y = 0.0;
    in.pose.orientation.z = 0.707;
    in.pose.orientation.w = 0.707;
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<geometry_msgs::msg::PoseStamped>(out_id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_EQ(out.header.stamp.sec, in.header.stamp.sec);
    EXPECT_DOUBLE_EQ(out.pose.position.x, in.pose.position.x);
    EXPECT_DOUBLE_EQ(out.pose.position.y, in.pose.position.y);
    EXPECT_DOUBLE_EQ(out.pose.orientation.z, in.pose.orientation.z);
    EXPECT_DOUBLE_EQ(out.pose.orientation.w, in.pose.orientation.w);
}

// ── sensor_msgs::msg::Imu ────────────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_Imu)
{
    sensor_msgs::msg::Imu in;
    in.header.frame_id = "imu_link";
    in.orientation.x = 0.0;
    in.orientation.y = 0.0;
    in.orientation.z = 0.0;
    in.orientation.w = 1.0;
    in.angular_velocity.x = 0.01;
    in.angular_velocity.y = -0.02;
    in.angular_velocity.z = 0.005;
    in.linear_acceleration.x = 0.12;
    in.linear_acceleration.y = -0.05;
    in.linear_acceleration.z = -9.81;
    in.orientation_covariance[0] = 0.001;
    in.angular_velocity_covariance[4] = 0.002;
    in.linear_acceleration_covariance[8] = 0.003;
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::Imu>(out_id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_DOUBLE_EQ(out.orientation.w, in.orientation.w);
    EXPECT_DOUBLE_EQ(out.angular_velocity.x, in.angular_velocity.x);
    EXPECT_DOUBLE_EQ(out.linear_acceleration.z, in.linear_acceleration.z);
    EXPECT_DOUBLE_EQ(out.orientation_covariance[0], in.orientation_covariance[0]);
    EXPECT_DOUBLE_EQ(out.angular_velocity_covariance[4], in.angular_velocity_covariance[4]);
    EXPECT_DOUBLE_EQ(out.linear_acceleration_covariance[8], in.linear_acceleration_covariance[8]);
}

// ── sensor_msgs::msg::JointState ─────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_JointState)
{
    sensor_msgs::msg::JointState in;
    in.header.frame_id = "robot_description";
    in.name = {"shoulder_pan", "shoulder_lift", "elbow_flex"};
    in.position = {0.5, -1.0, 0.785};
    in.velocity = {0.01, -0.02, 0.005};
    in.effort = {1.5, 2.0, 0.8};
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<sensor_msgs::msg::JointState>(out_id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    ASSERT_EQ(out.name.size(), in.name.size());
    EXPECT_EQ(out.name[0], in.name[0]);
    EXPECT_EQ(out.name[2], in.name[2]);
    ASSERT_EQ(out.position.size(), in.position.size());
    EXPECT_DOUBLE_EQ(out.position[0], in.position[0]);
    EXPECT_DOUBLE_EQ(out.position[2], in.position[2]);
    ASSERT_EQ(out.effort.size(), in.effort.size());
    EXPECT_DOUBLE_EQ(out.effort[1], in.effort[1]);
}

// ── nav_msgs::msg::Odometry ───────────────────────────────────────────────────

TEST_F(SoarRoundTripTest, RoundTrip_Odometry)
{
    nav_msgs::msg::Odometry in;
    in.header.frame_id = "odom";
    in.child_frame_id = "base_footprint";
    in.pose.pose.position.x = 5.0;
    in.pose.pose.position.y = 3.0;
    in.pose.pose.orientation.w = 1.0;
    in.pose.covariance[0] = 0.1;
    in.twist.twist.linear.x = 0.5;
    in.twist.twist.angular.z = 0.2;
    in.twist.covariance[35] = 0.05;
    auto *out_id = runThrough(in);
    ASSERT_NE(out_id, nullptr);
    auto out = soar_ros::msg::fromSoar<nav_msgs::msg::Odometry>(out_id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_EQ(out.child_frame_id, in.child_frame_id);
    EXPECT_DOUBLE_EQ(out.pose.pose.position.x, in.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(out.pose.pose.position.y, in.pose.pose.position.y);
    EXPECT_DOUBLE_EQ(out.pose.covariance[0], in.pose.covariance[0]);
    EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, in.twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, in.twist.twist.angular.z);
    EXPECT_DOUBLE_EQ(out.twist.covariance[35], in.twist.covariance[35]);
}
