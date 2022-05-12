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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

// #include "multi_nav2/Explored.hpp"
// #include "multi_nav2/SelectPF.hpp"
// #include "multi_nav2/Move.hpp"

// #include "nav2_msgs/action/navigate_to_pose.hpp"

// #include "multi_nav2/BTActionNode.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("patrolling_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("explored_bt_node"));
  factory.registerFromPlugin(loader.getOSName("move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("selectpf_bt_node"));

  float publisher_port_;
  float server_port_;

  node->declare_parameter("publisher_port", 1666.0);
  node->declare_parameter("server_port", 1667.0);

  node->get_parameter("publisher_port", publisher_port_);
  node->get_parameter("server_port", server_port_);


  std::string pkgpath = ament_index_cpp::get_package_share_directory("multi_nav2");
  std::string xml_file = pkgpath + "/multi_nav_tree/patrolling.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  // BT::Tree tree = factory.createTreeFromFile(xml_file);

  // BT::PublisherZMQ::PublisherZMQ(const BT::Tree & tree,unsigned max_msg_per_second = 25,unsigned publisher_port = 1666,unsigned server_port = 1667)
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, publisher_port_, server_port_);

  rclcpp::Rate rate(10);

  bool finish = false;

  while (!finish && rclcpp::ok()) {

    finish = tree.rootNode()->executeTick() == BT::NodeStatus::FAILURE;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  std::cout << "Patrolling finish !!" << std::endl;

  rclcpp::shutdown();
  return 0;
}
