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

#include "soar_ros/msg/diagnostic_msgs_converters.hpp"
#include "soar_ros/msg/tf2_msgs_converters.hpp"
#include "soar_ros/msg/trajectory_msgs_converters.hpp"
#include "soar_ros/msg/visualization_msgs_converters.hpp"

// ── Shared fixture ────────────────────────────────────────────────────────────

class MiscMsgsConverterTest : public ::testing::Test
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

// ── visualization_msgs / Marker ───────────────────────────────────────────────

TEST_F(MiscMsgsConverterTest, Marker_toSoar_Attributes)
{
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "map";
    msg.ns = "test_ns";
    msg.id = 42;
    msg.type = visualization_msgs::msg::Marker::SPHERE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.position.x = 1.0;
    msg.scale.x = 0.5;
    msg.color.r = 1.0f;
    msg.color.a = 1.0f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "m", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("ns"), "test_ns");
    EXPECT_STREQ(id->GetParameterValue("id"), "42");
    EXPECT_STREQ(id->GetParameterValue("type"), "2"); // SPHERE=2
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "pose"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "scale"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "color"), nullptr);
}

TEST_F(MiscMsgsConverterTest, Marker_RoundTrip)
{
    visualization_msgs::msg::Marker in;
    in.header.frame_id = "base_link";
    in.ns = "spheres";
    in.id = 7;
    in.type = visualization_msgs::msg::Marker::CUBE;
    in.action = visualization_msgs::msg::Marker::ADD;
    in.pose.position.x = 2.0;
    in.pose.position.y = -1.0;
    in.pose.orientation.w = 1.0;
    in.scale.x = 0.3;
    in.scale.y = 0.3;
    in.scale.z = 0.3;
    in.color.r = 0.0f;
    in.color.g = 1.0f;
    in.color.b = 0.0f;
    in.color.a = 0.8f;
    in.lifetime.sec = 0;
    in.lifetime.nanosec = 0;
    in.frame_locked = false;
    in.text = "hello";
    auto *id = soar_ros::msg::toSoar(inputLink(), "m", in);
    auto out = soar_ros::msg::fromSoar<visualization_msgs::msg::Marker>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    EXPECT_EQ(out.ns, in.ns);
    EXPECT_EQ(out.id, in.id);
    EXPECT_EQ(out.type, in.type);
    EXPECT_DOUBLE_EQ(out.pose.position.x, in.pose.position.x);
    EXPECT_FLOAT_EQ(out.color.g, in.color.g);
    EXPECT_FLOAT_EQ(out.color.a, in.color.a);
    EXPECT_EQ(out.text, in.text);
}

TEST_F(MiscMsgsConverterTest, Marker_WithPointsAndColors_RoundTrip)
{
    visualization_msgs::msg::Marker in;
    in.type = visualization_msgs::msg::Marker::LINE_STRIP;
    in.action = visualization_msgs::msg::Marker::ADD;
    in.pose.orientation.w = 1.0;
    in.scale.x = 0.05;
    for (int i = 0; i < 3; ++i)
    {
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(i);
        p.y = 0.0;
        p.z = 0.0;
        in.points.push_back(p);
        std_msgs::msg::ColorRGBA c;
        c.r = static_cast<float>(i) / 3.0f;
        c.g = 0.0f;
        c.b = 1.0f;
        c.a = 1.0f;
        in.colors.push_back(c);
    }
    auto *id = soar_ros::msg::toSoar(inputLink(), "m", in);
    EXPECT_STREQ(id->GetParameterValue("points_count"), "3");
    EXPECT_STREQ(id->GetParameterValue("colors_count"), "3");
    auto out = soar_ros::msg::fromSoar<visualization_msgs::msg::Marker>(id);
    ASSERT_EQ(out.points.size(), in.points.size());
    EXPECT_DOUBLE_EQ(out.points[2].x, in.points[2].x);
    ASSERT_EQ(out.colors.size(), in.colors.size());
    EXPECT_FLOAT_EQ(out.colors[1].r, in.colors[1].r);
}

// ── tf2_msgs / TFMessage ──────────────────────────────────────────────────────

TEST_F(MiscMsgsConverterTest, TFMessage_toSoar_Count)
{
    tf2_msgs::msg::TFMessage msg;
    for (int i = 0; i < 3; ++i)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = "world";
        tf.child_frame_id = "link_" + std::to_string(i);
        msg.transforms.push_back(tf);
    }
    auto *id = soar_ros::msg::toSoar(inputLink(), "tf", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("transforms_count"), "3");
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "transforms_0"), nullptr);
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "transforms_2"), nullptr);
}

TEST_F(MiscMsgsConverterTest, TFMessage_RoundTrip)
{
    tf2_msgs::msg::TFMessage in;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera_link";
    tf.transform.translation.x = 0.5;
    tf.transform.translation.z = 1.2;
    tf.transform.rotation.w = 1.0;
    in.transforms.push_back(tf);
    auto *id = soar_ros::msg::toSoar(inputLink(), "tf", in);
    auto out = soar_ros::msg::fromSoar<tf2_msgs::msg::TFMessage>(id);
    ASSERT_EQ(out.transforms.size(), 1u);
    EXPECT_EQ(out.transforms[0].header.frame_id, in.transforms[0].header.frame_id);
    EXPECT_EQ(out.transforms[0].child_frame_id, in.transforms[0].child_frame_id);
    EXPECT_DOUBLE_EQ(out.transforms[0].transform.translation.x, in.transforms[0].transform.translation.x);
    EXPECT_DOUBLE_EQ(out.transforms[0].transform.rotation.w, in.transforms[0].transform.rotation.w);
}

// ── diagnostic_msgs / KeyValue ────────────────────────────────────────────────

TEST_F(MiscMsgsConverterTest, KeyValue_RoundTrip)
{
    diagnostic_msgs::msg::KeyValue in;
    in.key = "temperature";
    in.value = "42.5";
    auto *id = soar_ros::msg::toSoar(inputLink(), "kv", in);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("key"), "temperature");
    EXPECT_STREQ(id->GetParameterValue("value"), "42.5");
    auto out = soar_ros::msg::fromSoar<diagnostic_msgs::msg::KeyValue>(id);
    EXPECT_EQ(out.key, in.key);
    EXPECT_EQ(out.value, in.value);
}

TEST_F(MiscMsgsConverterTest, DiagnosticStatus_RoundTrip)
{
    diagnostic_msgs::msg::DiagnosticStatus in;
    in.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    in.name = "battery";
    in.message = "Low charge";
    in.hardware_id = "bat0";
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "voltage";
    kv.value = "11.8";
    in.values.push_back(kv);
    kv.key = "current";
    kv.value = "2.1";
    in.values.push_back(kv);
    auto *id = soar_ros::msg::toSoar(inputLink(), "diag", in);
    EXPECT_STREQ(id->GetParameterValue("values_count"), "2");
    auto out = soar_ros::msg::fromSoar<diagnostic_msgs::msg::DiagnosticStatus>(id);
    EXPECT_EQ(out.level, in.level);
    EXPECT_EQ(out.name, in.name);
    EXPECT_EQ(out.message, in.message);
    ASSERT_EQ(out.values.size(), in.values.size());
    EXPECT_EQ(out.values[0].key, in.values[0].key);
    EXPECT_EQ(out.values[1].value, in.values[1].value);
}

TEST_F(MiscMsgsConverterTest, DiagnosticArray_RoundTrip)
{
    diagnostic_msgs::msg::DiagnosticArray in;
    in.header.frame_id = "diagnostics";
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    s.name = "motors";
    s.message = "All good";
    in.status.push_back(s);
    auto *id = soar_ros::msg::toSoar(inputLink(), "arr", in);
    auto out = soar_ros::msg::fromSoar<diagnostic_msgs::msg::DiagnosticArray>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    ASSERT_EQ(out.status.size(), 1u);
    EXPECT_EQ(out.status[0].level, in.status[0].level);
    EXPECT_EQ(out.status[0].name, in.status[0].name);
}

// ── trajectory_msgs / JointTrajectoryPoint ────────────────────────────────────

TEST_F(MiscMsgsConverterTest, JointTrajectoryPoint_RoundTrip)
{
    trajectory_msgs::msg::JointTrajectoryPoint in;
    in.positions = {0.1, 0.2, 0.3};
    in.velocities = {0.01, 0.02, 0.03};
    in.accelerations = {0.001, 0.002, 0.003};
    in.effort = {1.0, 2.0, 3.0};
    in.time_from_start.sec = 1;
    in.time_from_start.nanosec = 500000000;
    auto *id = soar_ros::msg::toSoar(inputLink(), "pt", in);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("positions_count"), "3");
    auto out = soar_ros::msg::fromSoar<trajectory_msgs::msg::JointTrajectoryPoint>(id);
    ASSERT_EQ(out.positions.size(), in.positions.size());
    EXPECT_DOUBLE_EQ(out.positions[2], in.positions[2]);
    ASSERT_EQ(out.velocities.size(), in.velocities.size());
    EXPECT_DOUBLE_EQ(out.velocities[0], in.velocities[0]);
    EXPECT_EQ(out.time_from_start.sec, in.time_from_start.sec);
    EXPECT_EQ(out.time_from_start.nanosec, in.time_from_start.nanosec);
}

TEST_F(MiscMsgsConverterTest, JointTrajectory_RoundTrip)
{
    trajectory_msgs::msg::JointTrajectory in;
    in.header.frame_id = "robot";
    in.joint_names = {"j1", "j2"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {0.0, 0.0};
    pt.time_from_start.sec = 0;
    in.points.push_back(pt);
    pt.positions = {1.0, -0.5};
    pt.time_from_start.sec = 2;
    in.points.push_back(pt);
    auto *id = soar_ros::msg::toSoar(inputLink(), "traj", in);
    EXPECT_STREQ(id->GetParameterValue("joint_names_count"), "2");
    EXPECT_STREQ(id->GetParameterValue("points_count"), "2");
    auto out = soar_ros::msg::fromSoar<trajectory_msgs::msg::JointTrajectory>(id);
    EXPECT_EQ(out.header.frame_id, in.header.frame_id);
    ASSERT_EQ(out.joint_names.size(), in.joint_names.size());
    EXPECT_EQ(out.joint_names[0], in.joint_names[0]);
    ASSERT_EQ(out.points.size(), in.points.size());
    EXPECT_DOUBLE_EQ(out.points[1].positions[0], in.points[1].positions[0]);
    EXPECT_EQ(out.points[1].time_from_start.sec, in.points[1].time_from_start.sec);
}
