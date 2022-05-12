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

#ifndef BEHAVIOR_TREE__SELECTPF_HPP_
#define BEHAVIOR_TREE__SELECTPF_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace multi_nav2
{

class SelectPF : public BT::ActionNodeBase
{
public:
  explicit SelectPF(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  void timer_callback_goal_marker();
  void callback_goal_poses_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void callback_robot_pos_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void callback_poses_(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint"),
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_goal_marker_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_pos_;
  geometry_msgs::msg::Pose robot_pos_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_poses_;
  geometry_msgs::msg::PoseStamped goal_pos_;

  bool other_robot_ = false;
  geometry_msgs::msg::PoseStamped goal_pos_other_;

  bool set_goal_ = false;

};

}  // namespace multi_nav2

#endif  // BEHAVIOR_TREE__SELECTPF_HPP_
