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


#include "multi_nav2/Explored.hpp"

using namespace std::placeholders;


namespace multi_nav2
{

Explored::Explored(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // se subscribe a las posiciones de los pf que publica check_pf.cpp
  sub_poses_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "poses", 10, std::bind(&Explored::callback_poses, this, _1));

}

void
Explored::halt()
{
  std::cout << "Explored halt" << std::endl;
}


BT::NodeStatus
Explored::tick()
{
  std::cout << "Explored tick" << std::endl;
  if (num_poses > 0) {
    // si todavía hay puntos desconocidos devuelvo success
    std::cout << "There are unknown points" << std::endl;
    num_poses = -1;
    return BT::NodeStatus::SUCCESS;
  } else if (num_poses == 0) {
    // si no quedan mas pf devuelvo fallo
    std::cout << "No unknown points left" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void
Explored::callback_poses(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  num_poses = int(msg->poses.size());
}


}  // namespace multi_nav2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multi_nav2::Explored>("Explored");
}
