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

#include "soar_ros/msg/nav_msgs_converters.hpp"

class NavMsgsConverterTest : public ::testing::Test
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

// ── MapMetaData ───────────────────────────────────────────────────────────────

TEST_F(NavMsgsConverterTest, MapMetaData_toSoar_Attributes)
{
    nav_msgs::msg::MapMetaData msg;
    msg.resolution = 0.05f;
    msg.width = 200;
    msg.height = 200;
    auto *id = soar_ros::msg::toSoar(inputLink(), "map", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("resolution", 0), nullptr);
    EXPECT_STREQ(id->GetParameterValue("width"), "200");
    EXPECT_STREQ(id->GetParameterValue("height"), "200");
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "origin"), nullptr);
}

TEST_F(NavMsgsConverterTest, MapMetaData_RoundTrip)
{
    nav_msgs::msg::MapMetaData in;
    in.map_load_time.sec = 100;
    in.map_load_time.nanosec = 0;
    in.resolution = 0.05f;
    in.width = 400;
    in.height = 300;
    in.origin.position.x = -10.0;
    in.origin.position.y = -7.5;
    auto *id = soar_ros::msg::toSoar(inputLink(), "map", in);
    auto out = soar_ros::msg::fromSoar<nav_msgs::msg::MapMetaData>(id);
    EXPECT_EQ(out.map_load_time.sec, in.map_load_time.sec);
    EXPECT_FLOAT_EQ(out.resolution, in.resolution);
    EXPECT_EQ(out.width, in.width);
    EXPECT_EQ(out.height, in.height);
    EXPECT_DOUBLE_EQ(out.origin.position.x, in.origin.position.x);
    EXPECT_DOUBLE_EQ(out.origin.position.y, in.origin.position.y);
}

// ── Odometry ──────────────────────────────────────────────────────────────────

TEST_F(NavMsgsConverterTest, Odometry_toSoar_Attributes)
{
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "odom", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("child_frame_id"), "base_link");
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "pose"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "twist"), nullptr);
}

TEST_F(NavMsgsConverterTest, Odometry_RoundTrip)
{
    nav_msgs::msg::Odometry in;
    in.header.frame_id = "odom";
    in.child_frame_id = "base_footprint";
    in.pose.pose.position.x = 3.5;
    in.pose.pose.position.y = -1.2;
    in.pose.pose.orientation.w = 1.0;
    in.pose.covariance[0] = 0.1;
    in.twist.twist.linear.x = 0.5;
    in.twist.twist.angular.z = 0.1;
    in.twist.covariance[35] = 0.05;
    auto *id = soar_ros::msg::toSoar(inputLink(), "odom", in);
    auto out = soar_ros::msg::fromSoar<nav_msgs::msg::Odometry>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_EQ(out.child_frame_id, in.child_frame_id);
    EXPECT_DOUBLE_EQ(out.pose.pose.position.x, in.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(out.pose.covariance[0], in.pose.covariance[0]);
    EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, in.twist.twist.linear.x);
    EXPECT_DOUBLE_EQ(out.twist.covariance[35], in.twist.covariance[35]);
}

// ── Path ──────────────────────────────────────────────────────────────────────

TEST_F(NavMsgsConverterTest, Path_toSoar_CountAttribute)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = 1.0;
    msg.poses.push_back(p);
    p.pose.position.x = 2.0;
    msg.poses.push_back(p);
    auto *id = soar_ros::msg::toSoar(inputLink(), "path", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("poses_count"), "2");
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "poses_0"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "poses_1"), nullptr);
}

TEST_F(NavMsgsConverterTest, Path_RoundTrip)
{
    nav_msgs::msg::Path in;
    in.header.frame_id = "map";
    for (int i = 0; i < 3; ++i)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = static_cast<double>(i);
        p.pose.position.y = static_cast<double>(i) * 0.5;
        p.pose.orientation.w = 1.0;
        in.poses.push_back(p);
    }
    auto *id = soar_ros::msg::toSoar(inputLink(), "path", in);
    auto out = soar_ros::msg::fromSoar<nav_msgs::msg::Path>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    ASSERT_EQ(out.poses.size(), in.poses.size());
    EXPECT_DOUBLE_EQ(out.poses[0].pose.position.x, in.poses[0].pose.position.x);
    EXPECT_DOUBLE_EQ(out.poses[2].pose.position.x, in.poses[2].pose.position.x);
    EXPECT_DOUBLE_EQ(out.poses[2].pose.position.y, in.poses[2].pose.position.y);
}
