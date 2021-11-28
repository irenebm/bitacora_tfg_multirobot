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

// --- ros2 run multi_nav2 border_points ---

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:
  MyNode(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name)
  {
    // se subscribe al mapa 
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MyNode::callback, this, _1));
    
    // publica visual markers
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mis_markers", 10);
    timer_ = create_wall_timer(
      rate, std::bind(&MyNode::timer_callback, this));
  }

  void timer_callback()
  {
    visualization_msgs::msg::MarkerArray total_markers_unknown_;
    visualization_msgs::msg::MarkerArray total_markers_unknown_limit_;

    for (int i = 0; i < int(pose_array_unknown_.poses.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.header.frame_id = "map";
      // marker.header.stamp = ros::Time::now();
      marker.id = i;
      marker.lifetime.sec = 5;
      marker.pose.position.x = pose_array_unknown_.poses[i].position.x;
      marker.pose.position.y = pose_array_unknown_.poses[i].position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0; // Don't forget to set the alpha! or your marker will be invisible!
      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;

      total_markers_unknown_.markers.push_back(marker);
    }
    for (int i = 0; i < int(pose_array_unknown_limit_.poses.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.header.frame_id = "map";
      // marker.header.stamp = ros::Time::now();
      marker.id = i + int(pose_array_unknown_.poses.size());
      marker.lifetime.sec = 5;
      marker.pose.position.x = pose_array_unknown_limit_.poses[i].position.x;
      marker.pose.position.y = pose_array_unknown_limit_.poses[i].position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0; // Don't forget to set the alpha! or your marker will be invisible!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      total_markers_unknown_limit_.markers.push_back(marker);
    }

    pub_->publish(total_markers_unknown_);
    pub_->publish(total_markers_unknown_limit_);

    pose_array_unknown_.poses.clear();
    pose_array_unknown_limit_.poses.clear();

    RCLCPP_INFO(this->get_logger(), "publicamos markers");
  }

  bool check_if_limit(float x, float y, int map_width, int map_height, nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
    // comprobamos que no vamos a ver un dato fuera del mapa 
    if(x + 1 < map_width) {
      int value_left = msg->data[map_width * (map_height - y - 1) + (x+1)];
      if(value_left != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    if(x - 1 > -1) {
      int value_right = msg->data[map_width * (map_height - y - 1) + (x-1)];
      if(value_right != -1) {
        return true;
      }
    }
    if(y + 1 < map_height) {
      int value_up = msg->data[map_width * (map_height - (y+1) - 1) + x];
      if(value_up != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    if(y - 1 > -1) {
      int value_down = msg->data[map_width * (map_height - (y-1) - 1) + x];
      if(value_down != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    return false; 
  }


private:
  void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
    pose_array_unknown_.poses.clear();
    pose_array_unknown_limit_.poses.clear();
    float map_resolution = msg->info.resolution;  // resolution [m/cell]
    int map_width = msg->info.width;    // ancho
    int map_height = msg->info.height;  // alto
    float origin_x = msg->info.origin.position.x;
    float origin_y = msg->info.origin.position.y;
    for(int x = 0; x < map_width; x++) {
      for(int y = 0; y < map_height; y++) {
        // size is (step * filas)
        // probabilities are in the range [0,100].  Unknown is -1.
        geometry_msgs::msg::Pose pose_;
        pose_.position.x = 1 * (x * map_resolution + origin_x);
        pose_.position.y = -1 * (y * map_resolution + origin_y);
        pose_.position.z = 0.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 1.0;
        int value = msg->data[map_width * (map_height - y - 1) + x];
        if(value == -1) {
          if (check_if_limit(x, y, map_width, map_height, msg)) {
            pose_array_unknown_limit_.poses.push_back(pose_);
          } else {
            pose_array_unknown_.poses.push_back(pose_);
          }       
        }
      }
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseArray pose_array_unknown_;
  geometry_msgs::msg::PoseArray pose_array_unknown_limit_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  geometry_msgs::msg::PoseArray pose_array_;

  auto node_A = std::make_shared<MyNode>("node_A", 1s);

  rclcpp::executors::SingleThreadedExecutor executor;
  
  executor.add_node(node_A);
  
  executor.spin();

  rclcpp::shutdown();

  return 0;
}