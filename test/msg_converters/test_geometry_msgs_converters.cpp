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

#include "soar_ros/msg/geometry_msgs_converters.hpp"

// ── Test fixture ──────────────────────────────────────────────────────────────

class GeoMsgsConverterTest : public ::testing::Test
{
protected:
    sml::Kernel *kernel{nullptr};
    sml::Agent *agent{nullptr};

    void SetUp() override
    {
        kernel = sml::Kernel::CreateKernelInCurrentThread();
        ASSERT_NE(kernel, nullptr);
        ASSERT_FALSE(kernel->HadError()) << kernel->GetLastErrorDescription();
        agent = kernel->CreateAgent("test_agent");
        ASSERT_NE(agent, nullptr);
        ASSERT_FALSE(kernel->HadError()) << kernel->GetLastErrorDescription();
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

    sml::Identifier *inputLink() { return agent->GetInputLink(); }
};

// ── Vector3 ───────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Vector3_toSoar_WmeAttributes)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = 1.0;
    msg.y = 2.0;
    msg.z = 3.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_NE(id->GetParameterValue("x"), nullptr);
    EXPECT_NE(id->GetParameterValue("y"), nullptr);
    EXPECT_NE(id->GetParameterValue("z"), nullptr);
}

TEST_F(GeoMsgsConverterTest, Vector3_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("v");
    id->CreateFloatWME("x", 1.5);
    id->CreateFloatWME("y", -2.5);
    id->CreateFloatWME("z", 0.0);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Vector3>(id);
    EXPECT_DOUBLE_EQ(result.x, 1.5);
    EXPECT_DOUBLE_EQ(result.y, -2.5);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST_F(GeoMsgsConverterTest, Vector3_Roundtrip)
{
    geometry_msgs::msg::Vector3 original;
    original.x = 1.1;
    original.y = -2.2;
    original.z = 3.3;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Vector3>(id);
    EXPECT_DOUBLE_EQ(result.x, original.x);
    EXPECT_DOUBLE_EQ(result.y, original.y);
    EXPECT_DOUBLE_EQ(result.z, original.z);
}

// ── Point ─────────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Point_Roundtrip)
{
    geometry_msgs::msg::Point original;
    original.x = 10.0;
    original.y = 20.0;
    original.z = -5.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "p", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Point>(id);
    EXPECT_DOUBLE_EQ(result.x, original.x);
    EXPECT_DOUBLE_EQ(result.y, original.y);
    EXPECT_DOUBLE_EQ(result.z, original.z);
}

// ── Point32 ───────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Point32_Roundtrip)
{
    geometry_msgs::msg::Point32 original;
    original.x = 1.5f;
    original.y = 2.5f;
    original.z = -3.5f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "p", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Point32>(id);
    EXPECT_NEAR(result.x, original.x, 1e-5f);
    EXPECT_NEAR(result.y, original.y, 1e-5f);
    EXPECT_NEAR(result.z, original.z, 1e-5f);
}

// ── Quaternion ────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Quaternion_toSoar_WmeAttributes)
{
    geometry_msgs::msg::Quaternion msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;
    msg.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "q", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_NE(id->GetParameterValue("x"), nullptr);
    EXPECT_NE(id->GetParameterValue("y"), nullptr);
    EXPECT_NE(id->GetParameterValue("z"), nullptr);
    EXPECT_NE(id->GetParameterValue("w"), nullptr);
}

TEST_F(GeoMsgsConverterTest, Quaternion_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("q");
    id->CreateFloatWME("x", 0.0);
    id->CreateFloatWME("y", 0.0);
    id->CreateFloatWME("z", 0.7071068);
    id->CreateFloatWME("w", 0.7071068);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Quaternion>(id);
    EXPECT_NEAR(result.z, 0.7071068, 1e-6);
    EXPECT_NEAR(result.w, 0.7071068, 1e-6);
}

TEST_F(GeoMsgsConverterTest, Quaternion_Roundtrip)
{
    geometry_msgs::msg::Quaternion original;
    original.x = 0.1;
    original.y = 0.2;
    original.z = 0.3;
    original.w = 0.9274;
    auto *id = soar_ros::msg::toSoar(inputLink(), "q", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Quaternion>(id);
    EXPECT_DOUBLE_EQ(result.x, original.x);
    EXPECT_DOUBLE_EQ(result.y, original.y);
    EXPECT_DOUBLE_EQ(result.z, original.z);
    EXPECT_DOUBLE_EQ(result.w, original.w);
}

// ── Pose ──────────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Pose_toSoar_ChildIds)
{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 1.0;
    msg.position.y = 2.0;
    msg.position.z = 3.0;
    msg.orientation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "pose", msg);
    ASSERT_NE(id, nullptr);
    // position and orientation must be child IDs
    auto *pos_wme = id->FindByAttribute("position", 0);
    ASSERT_NE(pos_wme, nullptr);
    ASSERT_NE(pos_wme->ConvertToIdentifier(), nullptr);
    auto *ori_wme = id->FindByAttribute("orientation", 0);
    ASSERT_NE(ori_wme, nullptr);
    ASSERT_NE(ori_wme->ConvertToIdentifier(), nullptr);
}

TEST_F(GeoMsgsConverterTest, Pose_toSoar_PositionValues)
{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 5.0;
    msg.position.y = -3.0;
    msg.position.z = 1.0;
    msg.orientation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "pose", msg);
    auto *pos = id->FindByAttribute("position", 0)->ConvertToIdentifier();
    ASSERT_NE(pos, nullptr);
    EXPECT_NE(pos->GetParameterValue("x"), nullptr);
    EXPECT_NE(pos->GetParameterValue("y"), nullptr);
    EXPECT_NE(pos->GetParameterValue("z"), nullptr);
}

TEST_F(GeoMsgsConverterTest, Pose_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("pose");
    auto *pos = id->CreateIdWME("position");
    pos->CreateFloatWME("x", 1.0);
    pos->CreateFloatWME("y", 2.0);
    pos->CreateFloatWME("z", 3.0);
    auto *ori = id->CreateIdWME("orientation");
    ori->CreateFloatWME("x", 0.0);
    ori->CreateFloatWME("y", 0.0);
    ori->CreateFloatWME("z", 0.0);
    ori->CreateFloatWME("w", 1.0);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Pose>(id);
    EXPECT_DOUBLE_EQ(result.position.x, 1.0);
    EXPECT_DOUBLE_EQ(result.position.y, 2.0);
    EXPECT_DOUBLE_EQ(result.position.z, 3.0);
    EXPECT_DOUBLE_EQ(result.orientation.w, 1.0);
}

TEST_F(GeoMsgsConverterTest, Pose_Roundtrip)
{
    geometry_msgs::msg::Pose original;
    original.position.x = 4.5;
    original.position.y = -1.2;
    original.position.z = 0.3;
    original.orientation.x = 0.0;
    original.orientation.y = 0.0;
    original.orientation.z = 0.5;
    original.orientation.w = 0.866025;
    auto *id = soar_ros::msg::toSoar(inputLink(), "pose", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Pose>(id);
    EXPECT_DOUBLE_EQ(result.position.x, original.position.x);
    EXPECT_DOUBLE_EQ(result.position.y, original.position.y);
    EXPECT_DOUBLE_EQ(result.position.z, original.position.z);
    EXPECT_DOUBLE_EQ(result.orientation.x, original.orientation.x);
    EXPECT_DOUBLE_EQ(result.orientation.y, original.orientation.y);
    EXPECT_DOUBLE_EQ(result.orientation.z, original.orientation.z);
    EXPECT_DOUBLE_EQ(result.orientation.w, original.orientation.w);
}

// ── PoseStamped ───────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, PoseStamped_toSoar_ChildIds)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp.sec = 1;
    msg.pose.position.x = 1.0;
    msg.pose.orientation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "ps", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("header", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("pose", 0), nullptr);
}

TEST_F(GeoMsgsConverterTest, PoseStamped_Roundtrip)
{
    geometry_msgs::msg::PoseStamped original;
    original.header.frame_id = "world";
    original.header.stamp.sec = 10;
    original.header.stamp.nanosec = 250000000u;
    original.pose.position.x = 1.0;
    original.pose.position.y = 2.0;
    original.pose.position.z = 3.0;
    original.pose.orientation.x = 0.0;
    original.pose.orientation.y = 0.0;
    original.pose.orientation.z = 0.0;
    original.pose.orientation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "ps", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::PoseStamped>(id);
    EXPECT_EQ(result.header.frame_id, original.header.frame_id);
    EXPECT_EQ(result.header.stamp.sec, original.header.stamp.sec);
    EXPECT_EQ(result.header.stamp.nanosec, original.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(result.pose.position.x, original.pose.position.x);
    EXPECT_DOUBLE_EQ(result.pose.position.y, original.pose.position.y);
    EXPECT_DOUBLE_EQ(result.pose.position.z, original.pose.position.z);
    EXPECT_DOUBLE_EQ(result.pose.orientation.w, original.pose.orientation.w);
}

// ── PoseWithCovariance ────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, PoseWithCovariance_Roundtrip)
{
    geometry_msgs::msg::PoseWithCovariance original;
    original.pose.position.x = 1.0;
    original.pose.position.y = 2.0;
    original.pose.position.z = 0.0;
    original.pose.orientation.w = 1.0;
    // Set diagonal of 6×6 covariance matrix
    for (std::size_t i = 0; i < 6; ++i)
    {
        original.covariance[i * 7] = static_cast<double>(i + 1) * 0.01;
    }
    auto *id = soar_ros::msg::toSoar(inputLink(), "pwc", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::PoseWithCovariance>(id);
    EXPECT_DOUBLE_EQ(result.pose.position.x, original.pose.position.x);
    for (std::size_t i = 0; i < 6; ++i)
    {
        EXPECT_DOUBLE_EQ(result.covariance[i * 7], original.covariance[i * 7]);
    }
}

// ── Twist ─────────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Twist_toSoar_ChildIds)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tw", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("linear", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("angular", 0), nullptr);
}

TEST_F(GeoMsgsConverterTest, Twist_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("tw");
    auto *lin = id->CreateIdWME("linear");
    lin->CreateFloatWME("x", 0.5);
    lin->CreateFloatWME("y", 0.0);
    lin->CreateFloatWME("z", 0.0);
    auto *ang = id->CreateIdWME("angular");
    ang->CreateFloatWME("x", 0.0);
    ang->CreateFloatWME("y", 0.0);
    ang->CreateFloatWME("z", 1.57);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Twist>(id);
    EXPECT_DOUBLE_EQ(result.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(result.angular.z, 1.57);
}

TEST_F(GeoMsgsConverterTest, Twist_Roundtrip)
{
    geometry_msgs::msg::Twist original;
    original.linear.x = 1.5;
    original.linear.y = 0.0;
    original.linear.z = -0.3;
    original.angular.x = 0.0;
    original.angular.y = 0.1;
    original.angular.z = 0.785;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tw", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Twist>(id);
    EXPECT_DOUBLE_EQ(result.linear.x, original.linear.x);
    EXPECT_DOUBLE_EQ(result.linear.y, original.linear.y);
    EXPECT_DOUBLE_EQ(result.linear.z, original.linear.z);
    EXPECT_DOUBLE_EQ(result.angular.x, original.angular.x);
    EXPECT_DOUBLE_EQ(result.angular.y, original.angular.y);
    EXPECT_DOUBLE_EQ(result.angular.z, original.angular.z);
}

// ── TwistStamped ──────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, TwistStamped_Roundtrip)
{
    geometry_msgs::msg::TwistStamped original;
    original.header.frame_id = "base_link";
    original.header.stamp.sec = 5;
    original.twist.linear.x = 2.0;
    original.twist.angular.z = 0.3;
    auto *id = soar_ros::msg::toSoar(inputLink(), "ts", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::TwistStamped>(id);
    EXPECT_EQ(result.header.frame_id, original.header.frame_id);
    EXPECT_EQ(result.header.stamp.sec, original.header.stamp.sec);
    EXPECT_DOUBLE_EQ(result.twist.linear.x, original.twist.linear.x);
    EXPECT_DOUBLE_EQ(result.twist.angular.z, original.twist.angular.z);
}

// ── Accel ─────────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Accel_Roundtrip)
{
    geometry_msgs::msg::Accel original;
    original.linear.x = 9.81;
    original.linear.y = 0.0;
    original.linear.z = 0.0;
    original.angular.x = 0.1;
    original.angular.y = 0.2;
    original.angular.z = 0.3;
    auto *id = soar_ros::msg::toSoar(inputLink(), "acc", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Accel>(id);
    EXPECT_DOUBLE_EQ(result.linear.x, original.linear.x);
    EXPECT_DOUBLE_EQ(result.angular.z, original.angular.z);
}

// ── Transform ─────────────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, Transform_toSoar_ChildIds)
{
    geometry_msgs::msg::Transform msg;
    msg.translation.x = 1.0;
    msg.rotation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tf", msg);
    ASSERT_NE(id, nullptr);
    ASSERT_NE(id->FindByAttribute("translation", 0), nullptr);
    ASSERT_NE(id->FindByAttribute("rotation", 0), nullptr);
}

TEST_F(GeoMsgsConverterTest, Transform_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("tf");
    auto *trans = id->CreateIdWME("translation");
    trans->CreateFloatWME("x", 1.0);
    trans->CreateFloatWME("y", 2.0);
    trans->CreateFloatWME("z", 0.5);
    auto *rot = id->CreateIdWME("rotation");
    rot->CreateFloatWME("x", 0.0);
    rot->CreateFloatWME("y", 0.0);
    rot->CreateFloatWME("z", 0.0);
    rot->CreateFloatWME("w", 1.0);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Transform>(id);
    EXPECT_DOUBLE_EQ(result.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(result.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(result.translation.z, 0.5);
    EXPECT_DOUBLE_EQ(result.rotation.w, 1.0);
}

TEST_F(GeoMsgsConverterTest, Transform_Roundtrip)
{
    geometry_msgs::msg::Transform original;
    original.translation.x = -1.0;
    original.translation.y = 2.5;
    original.translation.z = 0.75;
    original.rotation.x = 0.0;
    original.rotation.y = 0.0;
    original.rotation.z = 0.3826834;
    original.rotation.w = 0.9238795;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tf", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::Transform>(id);
    EXPECT_DOUBLE_EQ(result.translation.x, original.translation.x);
    EXPECT_DOUBLE_EQ(result.translation.y, original.translation.y);
    EXPECT_DOUBLE_EQ(result.translation.z, original.translation.z);
    EXPECT_DOUBLE_EQ(result.rotation.z, original.rotation.z);
    EXPECT_DOUBLE_EQ(result.rotation.w, original.rotation.w);
}

// ── TransformStamped ──────────────────────────────────────────────────────────

TEST_F(GeoMsgsConverterTest, TransformStamped_toSoar_ChildFrameId)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "world";
    msg.child_frame_id = "base_link";
    msg.transform.rotation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tfs", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("child_frame_id"), "base_link");
}

TEST_F(GeoMsgsConverterTest, TransformStamped_Roundtrip)
{
    geometry_msgs::msg::TransformStamped original;
    original.header.frame_id = "odom";
    original.header.stamp.sec = 99;
    original.header.stamp.nanosec = 500000000u;
    original.child_frame_id = "base_footprint";
    original.transform.translation.x = 3.0;
    original.transform.translation.y = -1.5;
    original.transform.translation.z = 0.0;
    original.transform.rotation.x = 0.0;
    original.transform.rotation.y = 0.0;
    original.transform.rotation.z = 0.0;
    original.transform.rotation.w = 1.0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "tfs", original);
    auto result = soar_ros::msg::fromSoar<geometry_msgs::msg::TransformStamped>(id);
    EXPECT_EQ(result.header.frame_id, original.header.frame_id);
    EXPECT_EQ(result.header.stamp.sec, original.header.stamp.sec);
    EXPECT_EQ(result.header.stamp.nanosec, original.header.stamp.nanosec);
    EXPECT_EQ(result.child_frame_id, original.child_frame_id);
    EXPECT_DOUBLE_EQ(result.transform.translation.x, original.transform.translation.x);
    EXPECT_DOUBLE_EQ(result.transform.translation.y, original.transform.translation.y);
    EXPECT_DOUBLE_EQ(result.transform.translation.z, original.transform.translation.z);
    EXPECT_DOUBLE_EQ(result.transform.rotation.w, original.transform.rotation.w);
}
