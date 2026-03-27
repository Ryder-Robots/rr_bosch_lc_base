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

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

#include <rr_bno055/transport_config.hpp>
#include <rr_bno055/transport_factory.hpp>
#include "rr_bosch_lc_base/rr_bosch_factory.hpp"

using namespace rr_bosch_lc;

struct Options
{
  std::string device = "/dev/i2c-1";
  uint8_t address = 0x28;
  rr_bno055::TransportType type = rr_bno055::TransportType::I2C;
  std::string ns = "sensors";
  std::vector<std::string> node_names{ "rr_bosch_imu_node" };
};

void print_usage(const char* prog)
{
  std::cout << "Usage: " << prog << " [OPTIONS]\n"
            << "\n"
            << "Options:\n"
            << "  --device PATH    I2C or UART device node  (default: /dev/i2c-1)\n"
            << "  --address ADDR   I2C hex address 0x28|0x29 (default: 0x28)\n"
            << "  --uart           Use UART transport instead of I2C\n"
            << "  --ns NAMESPACE   Namespace to use\n"
            << "  --nodename NODE  Repeatable arugment, need at least one per node\n"
            << "  --help           Show this message\n";
}

Options parse_args(int argc, char* argv[])
{
  Options opts;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help")
    {
      print_usage(argv[0]);
      std::exit(0);
    }
    else if (arg == "--device" && i + 1 < argc)
    {
      opts.device = argv[++i];
    }
    else if (arg == "--uart")
    {
      opts.type = rr_bno055::TransportType::UART;
      if (opts.device == "/dev/i2c-1")
      {
        opts.device = "/dev/ttyAMA0";
      }
    }
    else if (arg == "--address" && i + 1 < argc)
    {
      opts.address = static_cast<uint8_t>(std::stoul(argv[++i], nullptr, 16));
    }
    else if (arg == "--ns" && i + 1 < argc)
    {
      opts.ns = argv[++i];
    }
    else if (arg == "--nodename" && i + 1 < argc)
    {
      std::string node = argv[++i];
      if (std::find(opts.node_names.begin(), opts.node_names.end(), node) == opts.node_names.end())
      {
        opts.node_names.push_back(node);
      }
    }
    else
    {
      std::cerr << "Unknown option: " << arg << '\n';
      print_usage(argv[0]);
      std::exit(1);
    }
  }

  return opts;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const Options opts = parse_args(argc, argv);

  rr_bno055::RrBNO055Config::Builder b{};
  b.with_address(opts.address);
  b.with_device(opts.device);
  b.with_type(opts.type);
  auto tns_conf = std::make_shared<rr_bno055::RrBNO055Config>(b.build());

  rr_bno055::TransportFactory tfact;

  std::shared_ptr<rr_bno055::HardwareTransport> device_trns = tfact.get_or_create_transport(tns_conf);

  RrBoschFactory fact;
  std::vector<std::shared_ptr<RrBoschCommonSensor>> nodes = fact.get_nodes(opts.node_names, opts.ns);
  if (nodes.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "could not find any matching nodes");
    return 1;
  }

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  for (auto node : nodes)
  {
    node->set_transport(device_trns);
    executor->add_node(node->get_node_base_interface());
  }

  executor->spin();
  rclcpp::shutdown();
  device_trns.reset();
  return 0;
}