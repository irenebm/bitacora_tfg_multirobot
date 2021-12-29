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


#include "multi_nav2/Move.hpp"


namespace multi_nav2
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: multi_nav2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);
  RCLCPP_INFO(node_->get_logger(), "Move receiving goal: %f %f", goal.pose.position.x, goal.pose.position.y);

  goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
{
  std::cout << "Navigation succesful " << std::endl;

  return BT::NodeStatus::SUCCESS;
}


}  // namespace multi_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<multi_nav2::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<multi_nav2::Move>(
    "Move", builder);
}
