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
#include "rr_bosch_lc_base/rr_bosch_factory.hpp"
#include "rr_bosch_lc_base/rr_bosch_imu_node.hpp"

using namespace rr_bosch_lc;

// NOTE: Factory design — single concrete type (RrBoschImuNode)
//
// At this stage the factory produces only one concrete sensor type.
// A name-to-type map has been deliberately omitted until a second sensor
// type is introduced. Premature registration infrastructure would impose
// structure before the data has revealed what it needs to be.
//
// When a second concrete type is added, the correct pattern is:
//   - Introduce a std::map<std::string, FactoryFn> or equivalent registry
//   - Register each known name against its construction lambda
//   - Add the name to known_names() so the guard and log message
//     remain accurate automatically
//
// Do not add a map until there is a second type to put in it.

const std::vector<std::string> & RrBoschFactory::known_names()
{
  static const std::vector<std::string> names = {
    "rr_bosch_imu_node"
  };
  return names;
}

std::vector<std::shared_ptr<RrBoschCommonSensor>> RrBoschFactory::get_nodes(
  const std::vector<std::string> & node_names,
  const std::string & ns)
{
  std::vector<std::shared_ptr<RrBoschCommonSensor>> nodes;
  nodes.reserve(node_names.size());

  const auto & known = known_names();

  for (const auto & name : node_names) {
    if (std::find(known.begin(), known.end(), name) == known.end()) {
      std::ostringstream known_list;
      for (size_t i = 0; i < known.size(); ++i) {
        known_list << known[i];
        if (i + 1 < known.size()) {
          known_list << ", ";
        }
      }

      RCLCPP_ERROR(
        rclcpp::get_logger("RrBoschFactory"),
        "Unrecognised node name '%s'. Known names: [%s]. "
        "If adding a new sensor type, add it to known_names() "
        "and register it in get_nodes() in rr_bosch_factory.cpp.",
        name.c_str(), known_list.str().c_str());

      throw std::invalid_argument(
        "RrBoschFactory: unrecognised node name '" + name + "'.");
    }

    nodes.push_back(
      std::make_shared<RrBoschImuNode>(name, ns, rclcpp::NodeOptions()));
  }

  return nodes;
}
