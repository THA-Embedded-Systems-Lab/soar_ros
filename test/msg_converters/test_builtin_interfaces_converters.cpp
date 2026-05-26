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

#include "soar_ros/msg/builtin_interfaces_converters.hpp"

class BuiltinInterfacesConverterTest : public ::testing::Test
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
            kernel = nullptr;
            agent = nullptr;
        }
    }

    sml::Identifier *inputLink() { return agent->GetInputLink(); }
};

// ── Time ──────────────────────────────────────────────────────────────────────

TEST_F(BuiltinInterfacesConverterTest, Time_toSoar_Attributes)
{
    builtin_interfaces::msg::Time msg;
    msg.sec = 123;
    msg.nanosec = 456789;
    auto *id = soar_ros::msg::toSoar(inputLink(), "t", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("sec"), "123");
    EXPECT_STREQ(id->GetParameterValue("nanosec"), "456789");
}

TEST_F(BuiltinInterfacesConverterTest, Time_fromSoar)
{
    builtin_interfaces::msg::Time in;
    in.sec = 1000;
    in.nanosec = 999000000;
    auto *id = soar_ros::msg::toSoar(inputLink(), "t", in);
    auto out = soar_ros::msg::fromSoar<builtin_interfaces::msg::Time>(id);
    EXPECT_EQ(out.sec, in.sec);
    EXPECT_EQ(out.nanosec, in.nanosec);
}

TEST_F(BuiltinInterfacesConverterTest, Time_RoundTrip)
{
    builtin_interfaces::msg::Time in;
    in.sec = 1748260000;
    in.nanosec = 123456789;
    auto *id = soar_ros::msg::toSoar(inputLink(), "t", in);
    auto out = soar_ros::msg::fromSoar<builtin_interfaces::msg::Time>(id);
    EXPECT_EQ(out.sec, in.sec);
    EXPECT_EQ(out.nanosec, in.nanosec);
}

// ── Duration ──────────────────────────────────────────────────────────────────

TEST_F(BuiltinInterfacesConverterTest, Duration_toSoar_Attributes)
{
    builtin_interfaces::msg::Duration msg;
    msg.sec = -5;
    msg.nanosec = 500000000;
    auto *id = soar_ros::msg::toSoar(inputLink(), "d", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("sec"), "-5");
    EXPECT_STREQ(id->GetParameterValue("nanosec"), "500000000");
}

TEST_F(BuiltinInterfacesConverterTest, Duration_RoundTrip)
{
    builtin_interfaces::msg::Duration in;
    in.sec = 10;
    in.nanosec = 250000000;
    auto *id = soar_ros::msg::toSoar(inputLink(), "d", in);
    auto out = soar_ros::msg::fromSoar<builtin_interfaces::msg::Duration>(id);
    EXPECT_EQ(out.sec, in.sec);
    EXPECT_EQ(out.nanosec, in.nanosec);
}
