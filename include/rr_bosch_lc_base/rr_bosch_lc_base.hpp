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

#pragma once

#include <vector>
#include <string>
#include <memory>
#include "nav2_util/lifecycle_node.hpp"
#include "rr_bno055/hardware_transport.hpp"
#include "rr_bosch_lc_base/rr_bosch_common_sensor.hpp"

namespace rr_bosch_lc
{
class RrBoschLcBase : public nav2_util::LifecycleNode
{
protected:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

public:
  explicit RrBoschLcBase(
    const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & options)
  : nav2_util::LifecycleNode(node_name, ns, options)
  {
  }

  virtual ~RrBoschLcBase() = default;

  CallbackReturn on_configure(const State & state) override;

  CallbackReturn on_activate(const State & state) override;

  CallbackReturn on_deactivate(const State & state) override;

  CallbackReturn on_cleanup(const State & state) override;

  CallbackReturn on_shutdown(const State & state) override;

private:
  std::vector<std::shared_ptr<RrBoschCommonSensor>> nodes_;
  std::shared_ptr<rr_bno055::HardwareTransport> device_trns_;
};
}  // namespace rr_bosch_lc
