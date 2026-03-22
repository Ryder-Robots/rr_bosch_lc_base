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
#include "rr_bno055/bno055_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rr_bosch_lc_base/rr_bosch_common_sensor.hpp"
#include <thread>
#include <chrono>
#include <functional>

namespace rr_bosch_lc
{

class RrBoschImuNode : public RrBoschCommonSensor
{
protected:
  using CallbackReturn = RrBoschCommonSensor::CallbackReturn;
  using State = RrBoschCommonSensor::State;

public:
  explicit RrBoschImuNode(const std::string& node_name, const std::string& ns, const rclcpp::NodeOptions& options)
    : RrBoschCommonSensor(node_name, ns, options)
  {
    // publisher
    declare_parameter("frame_id", "imu_link");
    declare_parameter("publish_frequency", 20.0);
    declare_parameter("imu_topic", "/imu/data");

    // transport
    declare_parameter("transport_protocol", "i2c");  // "i2c" or "uart"
    declare_parameter("device", "/dev/i2c-1");

    // i2c-specific
    declare_parameter("i2c_address", 0x28);

    // axis mounting
    declare_parameter("axis_remap", static_cast<int>(rr_bno055::RRBNO055_REMAP_X_Y));
    declare_parameter("axis_sign_x", static_cast<int>(rr_bno055::RRBNO055_REMAP_AXIS_NEGATIVE));
    declare_parameter("axis_sign_y", static_cast<int>(rr_bno055::RRBNO055_REMAP_AXIS_POSITIVE));
    declare_parameter("axis_sign_z", static_cast<int>(rr_bno055::RRBNO055_REMAP_AXIS_NEGATIVE));
  }

  virtual ~RrBoschImuNode() = default;

  CallbackReturn on_configure(const State& state) override;
  CallbackReturn on_activate(const State& state) override;
  CallbackReturn on_deactivate(const State& state) override;
  CallbackReturn on_cleanup(const State& state) override;
  CallbackReturn on_shutdown(const State& state) override;

  static constexpr int MAX_POLLS = 180;  // 90 seconds at 500 ms/poll
  static constexpr int POLL_INTERVAL_MS = 500;
  static constexpr int MAX_CONSECUTIVE_FAILURES = 3;

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string frame_id_;
  double publish_frequency_;
  rr_bno055::Bno055Device device_;
  std::shared_ptr<rr_bno055::RrBNO055Config> conf_;

  void publish_callback_();
  int consecutive_failures_ = 0;
};

}  // namespace rr_bosch_lc