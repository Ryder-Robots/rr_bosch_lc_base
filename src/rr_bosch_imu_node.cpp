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

#include "rr_bosch_lc_base/rr_bosch_imu_node.hpp"

using namespace rr_bosch_lc;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

CallbackReturn RrBoschImuNode::on_configure(const State& state)
{
  RCLCPP_INFO(this->get_logger(), "configuring %s", this->get_name());
  if (!device_trns_)
  {
    RCLCPP_ERROR(get_logger(), "RrBoschImuNode: configure: transport driver needs to be set before it can be created");
    return CallbackReturn::FAILURE;
  }

  auto protocol = get_parameter("transport_protocol").as_string();

  rr_bno055::RrBNO055Config::Builder b{};
  b.with_device(get_parameter("device").as_string());
  b.with_axis_remap(static_cast<rr_bno055::RrBno055AxisRemap>(get_parameter("axis_remap").as_int()));
  b.with_axis_sign_xyz({ static_cast<rr_bno055::RrBno055AxisSign>(get_parameter("axis_sign_x").as_int()),
                         static_cast<rr_bno055::RrBno055AxisSign>(get_parameter("axis_sign_y").as_int()),
                         static_cast<rr_bno055::RrBno055AxisSign>(get_parameter("axis_sign_z").as_int()) });

  if (protocol == "i2c")
  {
    // read i2c_address, build config with address
    auto address = static_cast<uint8_t>(get_parameter("i2c_address").as_int());
    b.with_address(address);
  }
  else if (protocol == "uart")
  {
    // UART doesn't need an address — build config differently
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "unknown transport_protocol '%s'", protocol.c_str());
    return CallbackReturn::FAILURE;
  }

  conf_ = std::make_shared<rr_bno055::RrBNO055Config>(b.build());
  return CallbackReturn::SUCCESS;
}

// Transport is checked in both on_configure and on_activate deliberately.
// Under the ROS2 lifecycle manager this is redundant — the state machine
// prevents activation without a prior successful configure. However
// RrBoschImuNode may be instantiated and driven manually outside the
// lifecycle manager, for example in a test harness or as a plain object.
// In that context on_activate is the only guard that exists. Do not remove
// the check from on_activate on the assumption that configure has always
// run first — that assumption only holds under managed lifecycle.
CallbackReturn RrBoschImuNode::on_activate(const State& state)
{
  RCLCPP_INFO(this->get_logger(), "activating %s", this->get_name());
  if (!conf_)
  {
    RCLCPP_ERROR(get_logger(), "RrBoschImuNode: activate: configuration is null");
    return CallbackReturn::FAILURE;
  }
  if (!device_trns_)
  {
    RCLCPP_ERROR(get_logger(), "RrBoschImuNode: activate: driver needs to be set before it can be created");
    return CallbackReturn::FAILURE;
  }

  if (!device_.initialize(conf_, device_trns_))
  {
    RCLCPP_ERROR(get_logger(), "RrBoschImuNode: activate: could not initilize bno055");
    return CallbackReturn::FAILURE;
  }

  if (!device_.set_op_mode(rr_bno055::RRBNO055_OPERATION_MODE_NDOF))
  {
    RCLCPP_ERROR(get_logger(), "%s: activate: failed to set NDOF operation mode", get_name());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "%s: activate: calibrating", this->get_name());
  uint8_t calib_status;
  int polls = 0;
  while (!device_.is_fully_calibrated(calib_status))
  {
    if (++polls > MAX_POLLS)
    {
      RCLCPP_ERROR(get_logger(), "%s: activate: calibration timed out after %d s (status=0x%02x)", get_name(),
                   (MAX_POLLS * POLL_INTERVAL_MS) / 1000, calib_status);
      return CallbackReturn::FAILURE;
    }
    // Log which sensors are still pending
    if (calib_status & rr_bno055::SYS_NOT_CALIBRATED)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: waiting on system calibration", get_name());
    }
    if (calib_status & rr_bno055::GYRO_NOT_CALIBRATED)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: waiting on gyro — keep sensor still", get_name());
    }
    if (calib_status & rr_bno055::ACCEL_NOT_CALIBRATED)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: waiting on accel — place in 6 static positions",
                           get_name());
    }
    if (calib_status & rr_bno055::MAG_NOT_CALIBRATED)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: waiting on mag — move sensor in figure-8",
                           get_name());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
  }
  // set up publisher.
  try
  {
    auto publish_callback = std::bind(&RrBoschImuNode::publish_callback_, this);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), 10);
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(250), publish_callback);
    imu_pub_->on_activate();
  }
  catch (const rclcpp::exceptions::RCLError& e)
  {
    // not worried if the device is not deinitlized, we just want to try
    device_.deinitialize();
    RCLCPP_ERROR(get_logger(), "%s: failed to create IMU publisher: %s", get_name(), e.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void RrBoschImuNode::publish_callback_()
{
  bno055_quaternion_t quat{};
  bno055_gyro_t gyro{};
  bno055_linear_accel_t accel{};

  bool read_ok =
      device_.read_quaternion(quat) && device_.read_angular_velocity(gyro) && device_.read_linear_acceleration(accel);

  if (!read_ok)
  {
    if (++consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES)
    {
      RCLCPP_ERROR(get_logger(),
                   "%s: publish_callback_: device unavailable after %d consecutive failures — "
                   "cancelling publisher. Node must be restarted to recover.",
                   get_name(), consecutive_failures_);
      publish_timer_->cancel();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%s: publish_callback_: read failure %d of %d before shutdown", get_name(),
                           consecutive_failures_, MAX_CONSECUTIVE_FAILURES);
    }
    return;
  }

  // reset on successful read
  consecutive_failures_ = 0;

  static constexpr double QUAT_SCALE = 1.0 / 16384.0;
  static constexpr double GYRO_SCALE = (1.0 / 16.0) * (M_PI / 180.0);
  static constexpr double ACCEL_SCALE = 1.0 / 100.0;

  sensor_msgs::msg::Imu msg{};
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = frame_id_;

  msg.orientation.x = quat.x * QUAT_SCALE;
  msg.orientation.y = quat.y * QUAT_SCALE;
  msg.orientation.z = quat.z * QUAT_SCALE;
  msg.orientation.w = quat.w * QUAT_SCALE;

  msg.angular_velocity.x = gyro.x * GYRO_SCALE;
  msg.angular_velocity.y = gyro.y * GYRO_SCALE;
  msg.angular_velocity.z = gyro.z * GYRO_SCALE;

  msg.linear_acceleration.x = accel.x * ACCEL_SCALE;
  msg.linear_acceleration.y = accel.y * ACCEL_SCALE;
  msg.linear_acceleration.z = accel.z * ACCEL_SCALE;

  // Covariance matrices left as zero-initialised — unknown at this stage.
  // If covariance estimates become available from calibration data they
  // should be populated here before publishing.

  imu_pub_->publish(msg);
}