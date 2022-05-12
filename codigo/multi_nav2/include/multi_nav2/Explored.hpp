// Copyright 2021 Irene Bandera Moreno
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

#ifndef BEHAVIOR_TREE__EXPLORED_HPP_
#define BEHAVIOR_TREE__EXPLORED_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>


#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace multi_nav2
{

class Explored : public BT::ActionNodeBase
{
public:
  explicit Explored(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  void callback_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_poses_;

  int num_poses = -1;

};

}  // namespace multi_nav2

#endif  // BEHAVIOR_TREE__EXPLORED_HPP_
