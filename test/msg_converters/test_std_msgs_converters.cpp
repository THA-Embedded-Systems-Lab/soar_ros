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

#include "soar_ros/msg/std_msgs_converters.hpp"

// ── Test fixture ──────────────────────────────────────────────────────────────

class StdMsgsConverterTest : public ::testing::Test
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

// ── Bool ──────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Bool_toSoar_True)
{
    std_msgs::msg::Bool msg;
    msg.data = true;
    auto *id = soar_ros::msg::toSoar(inputLink(), "b", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("data"), "1");
}

TEST_F(StdMsgsConverterTest, Bool_toSoar_False)
{
    std_msgs::msg::Bool msg;
    msg.data = false;
    auto *id = soar_ros::msg::toSoar(inputLink(), "b", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("data"), "0");
}

TEST_F(StdMsgsConverterTest, Bool_fromSoar_True)
{
    auto *id = inputLink()->CreateIdWME("b");
    id->CreateIntWME("data", 1);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Bool>(id);
    EXPECT_TRUE(result.data);
}

TEST_F(StdMsgsConverterTest, Bool_fromSoar_False)
{
    auto *id = inputLink()->CreateIdWME("b");
    id->CreateIntWME("data", 0);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Bool>(id);
    EXPECT_FALSE(result.data);
}

TEST_F(StdMsgsConverterTest, Bool_Roundtrip)
{
    std_msgs::msg::Bool original;
    original.data = true;
    auto *id = soar_ros::msg::toSoar(inputLink(), "b", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Bool>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Int8 ──────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Int8_Roundtrip)
{
    std_msgs::msg::Int8 original;
    original.data = -42;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Int8>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Int16 ─────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Int16_Roundtrip)
{
    std_msgs::msg::Int16 original;
    original.data = -1000;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Int16>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Int32 ─────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Int32_toSoar_WmeAttribute)
{
    std_msgs::msg::Int32 msg;
    msg.data = 99;
    auto *id = soar_ros::msg::toSoar(inputLink(), "n", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("data"), "99");
}

TEST_F(StdMsgsConverterTest, Int32_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("n");
    id->CreateIntWME("data", 99);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Int32>(id);
    EXPECT_EQ(result.data, 99);
}

TEST_F(StdMsgsConverterTest, Int32_Roundtrip_Negative)
{
    std_msgs::msg::Int32 original;
    original.data = -2147483647;
    auto *id = soar_ros::msg::toSoar(inputLink(), "n", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Int32>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Int64 ─────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Int64_Roundtrip)
{
    std_msgs::msg::Int64 original;
    original.data = -9000000000LL;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Int64>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── UInt8 ─────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, UInt8_Roundtrip)
{
    std_msgs::msg::UInt8 original;
    original.data = 255;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::UInt8>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── UInt16 ────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, UInt16_Roundtrip)
{
    std_msgs::msg::UInt16 original;
    original.data = 65535;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::UInt16>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── UInt32 ────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, UInt32_Roundtrip)
{
    std_msgs::msg::UInt32 original;
    original.data = 4000000000u;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::UInt32>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── UInt64 ────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, UInt64_Roundtrip)
{
    std_msgs::msg::UInt64 original;
    original.data = 123456789012345ULL;
    auto *id = soar_ros::msg::toSoar(inputLink(), "v", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::UInt64>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Float32 ───────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Float32_toSoar_WmeAttribute)
{
    std_msgs::msg::Float32 msg;
    msg.data = 3.14f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "f", msg);
    ASSERT_NE(id, nullptr);
    // The WME attribute "data" must exist
    EXPECT_NE(id->GetParameterValue("data"), nullptr);
}

TEST_F(StdMsgsConverterTest, Float32_Roundtrip)
{
    std_msgs::msg::Float32 original;
    original.data = 2.718f;
    auto *id = soar_ros::msg::toSoar(inputLink(), "f", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Float32>(id);
    EXPECT_NEAR(static_cast<double>(result.data), static_cast<double>(original.data), 1e-5);
}

// ── Float64 ───────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Float64_toSoar_WmeAttribute)
{
    std_msgs::msg::Float64 msg;
    msg.data = 1.23456789;
    auto *id = soar_ros::msg::toSoar(inputLink(), "f", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_NE(id->GetParameterValue("data"), nullptr);
}

TEST_F(StdMsgsConverterTest, Float64_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("f");
    id->CreateFloatWME("data", 9.81);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Float64>(id);
    EXPECT_DOUBLE_EQ(result.data, 9.81);
}

TEST_F(StdMsgsConverterTest, Float64_Roundtrip)
{
    std_msgs::msg::Float64 original;
    original.data = -273.15;
    auto *id = soar_ros::msg::toSoar(inputLink(), "f", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Float64>(id);
    EXPECT_DOUBLE_EQ(result.data, original.data);
}

// ── String ────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, String_toSoar_WmeAttribute)
{
    std_msgs::msg::String msg;
    msg.data = "hello soar";
    auto *id = soar_ros::msg::toSoar(inputLink(), "s", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("data"), "hello soar");
}

TEST_F(StdMsgsConverterTest, String_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("s");
    id->CreateStringWME("data", "world");
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::String>(id);
    EXPECT_EQ(result.data, "world");
}

TEST_F(StdMsgsConverterTest, String_Roundtrip)
{
    std_msgs::msg::String original;
    original.data = "roundtrip test";
    auto *id = soar_ros::msg::toSoar(inputLink(), "s", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::String>(id);
    EXPECT_EQ(result.data, original.data);
}

TEST_F(StdMsgsConverterTest, String_Empty)
{
    std_msgs::msg::String original;
    original.data = "";
    auto *id = soar_ros::msg::toSoar(inputLink(), "s", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::String>(id);
    EXPECT_EQ(result.data, original.data);
}

// ── Header ────────────────────────────────────────────────────────────────────

TEST_F(StdMsgsConverterTest, Header_toSoar_WmeStructure)
{
    std_msgs::msg::Header msg;
    msg.frame_id = "base_link";
    msg.stamp.sec = 100;
    msg.stamp.nanosec = 500000000u;
    auto *id = soar_ros::msg::toSoar(inputLink(), "h", msg);
    ASSERT_NE(id, nullptr);
    EXPECT_STREQ(id->GetParameterValue("frame_id"), "base_link");
    // stamp child must be an ID
    auto *stamp_wme = id->FindByAttribute("stamp", 0);
    ASSERT_NE(stamp_wme, nullptr);
    auto *stamp = stamp_wme->ConvertToIdentifier();
    ASSERT_NE(stamp, nullptr);
    EXPECT_STREQ(stamp->GetParameterValue("sec"), "100");
    EXPECT_STREQ(stamp->GetParameterValue("nanosec"), "500000000");
}

TEST_F(StdMsgsConverterTest, Header_fromSoar)
{
    auto *id = inputLink()->CreateIdWME("h");
    id->CreateStringWME("frame_id", "map");
    auto *stamp = id->CreateIdWME("stamp");
    stamp->CreateIntWME("sec", 42);
    stamp->CreateIntWME("nanosec", 123456789);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Header>(id);
    EXPECT_EQ(result.frame_id, "map");
    EXPECT_EQ(result.stamp.sec, 42);
    EXPECT_EQ(result.stamp.nanosec, 123456789u);
}

TEST_F(StdMsgsConverterTest, Header_Roundtrip)
{
    std_msgs::msg::Header original;
    original.frame_id = "odom";
    original.stamp.sec = 1000;
    original.stamp.nanosec = 999999999u;
    auto *id = soar_ros::msg::toSoar(inputLink(), "h", original);
    auto result = soar_ros::msg::fromSoar<std_msgs::msg::Header>(id);
    EXPECT_EQ(result.frame_id, original.frame_id);
    EXPECT_EQ(result.stamp.sec, original.stamp.sec);
    EXPECT_EQ(result.stamp.nanosec, original.stamp.nanosec);
}
