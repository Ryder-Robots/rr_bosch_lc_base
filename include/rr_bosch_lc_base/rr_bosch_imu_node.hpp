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

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rr_bosch_lc_base/rr_bosch_common_sensor.hpp"

namespace rr_bosch_lc
{

class RrBoschImuNode : public RrBoschCommonSensor
{
protected:
  using CallbackReturn = RrBoschCommonSensor::CallbackReturn;
  using State = RrBoschCommonSensor::State;

public:
  explicit RrBoschImuNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
  : RrBoschCommonSensor(node_name, ns, options)
  {
    declare_parameter("frame_id", "imu_link");
    declare_parameter("publish_frequency", 20.0);
  }

  virtual ~RrBoschImuNode() = default;

  CallbackReturn on_configure(const State & state) override;
  CallbackReturn on_activate(const State & state) override;
  CallbackReturn on_deactivate(const State & state) override;
  CallbackReturn on_cleanup(const State & state) override;
  CallbackReturn on_shutdown(const State & state) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string frame_id_;
  double publish_frequency_;
};

}  // namespace rr_bosch_lc