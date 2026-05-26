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

#include "soar_ros/msg/action_msgs_converters.hpp"

class ActionMsgsConverterTest : public ::testing::Test
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

// ── GoalInfo ──────────────────────────────────────────────────────────────────

TEST_F(ActionMsgsConverterTest, GoalInfo_toSoar_Attributes)
{
    action_msgs::msg::GoalInfo msg;
    // uuid: 16 bytes
    for (uint8_t i = 0; i < 16; ++i)
    {
        msg.goal_id.uuid[i] = i;
    }
    msg.stamp.sec = 10;
    msg.stamp.nanosec = 0;
    auto *id = soar_ros::msg::toSoar(inputLink(), "gi", msg);
    ASSERT_NE(id, nullptr);
    // uuid bytes are stored under a nested goal_id identifier
    auto *uuid_id = soar_ros::msg::detail::getChild(id, "goal_id");
    ASSERT_NE(uuid_id, nullptr);
    EXPECT_STREQ(uuid_id->GetParameterValue("uuid_0"), "0");
    EXPECT_STREQ(uuid_id->GetParameterValue("uuid_15"), "15");
    ASSERT_NE(soar_ros::msg::detail::getChild(id, "stamp"), nullptr);
}

TEST_F(ActionMsgsConverterTest, GoalInfo_RoundTrip)
{
    action_msgs::msg::GoalInfo in;
    for (uint8_t i = 0; i < 16; ++i)
    {
        in.goal_id.uuid[i] = static_cast<uint8_t>(i * 10);
    }
    in.stamp.sec = 42;
    in.stamp.nanosec = 100;
    auto *id = soar_ros::msg::toSoar(inputLink(), "gi", in);
    auto out = soar_ros::msg::fromSoar<action_msgs::msg::GoalInfo>(id);
    for (int i = 0; i < 16; ++i)
    {
        EXPECT_EQ(out.goal_id.uuid[i], in.goal_id.uuid[i]) << "uuid[" << i << "] mismatch";
    }
    EXPECT_EQ(out.stamp.sec, in.stamp.sec);
    EXPECT_EQ(out.stamp.nanosec, in.stamp.nanosec);
}

// ── GoalStatus ────────────────────────────────────────────────────────────────

TEST_F(ActionMsgsConverterTest, GoalStatus_RoundTrip)
{
    action_msgs::msg::GoalStatus in;
    for (uint8_t i = 0; i < 16; ++i)
    {
        in.goal_info.goal_id.uuid[i] = i;
    }
    in.goal_info.stamp.sec = 5;
    in.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    auto *id = soar_ros::msg::toSoar(inputLink(), "gs", in);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("status"), "2"); // STATUS_EXECUTING=2
    auto out = soar_ros::msg::fromSoar<action_msgs::msg::GoalStatus>(id);
    EXPECT_EQ(out.status, in.status);
    EXPECT_EQ(out.goal_info.stamp.sec, in.goal_info.stamp.sec);
    for (int i = 0; i < 16; ++i)
    {
        EXPECT_EQ(out.goal_info.goal_id.uuid[i], in.goal_info.goal_id.uuid[i]);
    }
}

// ── GoalStatusArray ───────────────────────────────────────────────────────────

TEST_F(ActionMsgsConverterTest, GoalStatusArray_RoundTrip)
{
    action_msgs::msg::GoalStatusArray in;
    for (int g = 0; g < 2; ++g)
    {
        action_msgs::msg::GoalStatus gs;
        for (uint8_t i = 0; i < 16; ++i)
        {
            gs.goal_info.goal_id.uuid[i] = static_cast<uint8_t>(g * 16 + i);
        }
        gs.status = static_cast<int8_t>(g + 1);
        in.status_list.push_back(gs);
    }
    auto *id = soar_ros::msg::toSoar(inputLink(), "gsa", in);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("status_list_count"), "2");
    auto out = soar_ros::msg::fromSoar<action_msgs::msg::GoalStatusArray>(id);
    ASSERT_EQ(out.status_list.size(), in.status_list.size());
    EXPECT_EQ(out.status_list[0].status, in.status_list[0].status);
    EXPECT_EQ(out.status_list[1].status, in.status_list[1].status);
    EXPECT_EQ(out.status_list[1].goal_info.goal_id.uuid[15], in.status_list[1].goal_info.goal_id.uuid[15]);
}
