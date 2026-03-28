// Copyright (c) 2026 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rr_bosch_lc_base/rr_bosch_factory.hpp"

class RrBoschFactoryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  rr_bosch_lc::RrBoschFactory factory_;
};

TEST_F(RrBoschFactoryTest, KnownNameReturnsOneNode)
{
  auto nodes = factory_.get_nodes({"rr_bosch_imu_node"}, "sensors");
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_NE(nodes[0], nullptr);
}

TEST_F(RrBoschFactoryTest, KnownNameNodeHasCorrectName)
{
  auto nodes = factory_.get_nodes({"rr_bosch_imu_node"}, "sensors");
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[0]->get_name(), std::string("rr_bosch_imu_node"));
}

TEST_F(RrBoschFactoryTest, NamespacePropagatesToNode)
{
  auto nodes = factory_.get_nodes({"rr_bosch_imu_node"}, "test_ns");
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[0]->get_namespace(), std::string("/test_ns"));
}

TEST_F(RrBoschFactoryTest, EmptyInputReturnsEmptyVector)
{
  auto nodes = factory_.get_nodes({}, "sensors");
  EXPECT_TRUE(nodes.empty());
}

TEST_F(RrBoschFactoryTest, MultipleKnownNamesReturnsCorrectCount)
{
  auto nodes = factory_.get_nodes({"rr_bosch_imu_node", "rr_bosch_imu_node"}, "sensors");
  EXPECT_EQ(nodes.size(), 2u);
}

TEST_F(RrBoschFactoryTest, UnknownNameThrowsInvalidArgument)
{
  EXPECT_THROW(
    factory_.get_nodes({"not_a_real_node"}, "sensors"),
    std::invalid_argument);
}

TEST_F(RrBoschFactoryTest, UnknownNameInMixedListThrows)
{
  EXPECT_THROW(
    factory_.get_nodes({"rr_bosch_imu_node", "not_a_real_node"}, "sensors"),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
