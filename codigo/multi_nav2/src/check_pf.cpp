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

// --- ros2 run multi_nav2 CheckPF ---

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"


#include <memory>
#include <cmath>
#include <thread>
#include <functional>


using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:
  // using NavigateToPose = nav2_msgs::action::NavigateToPose;
  // using GoalHandleNavigateToPose_s_ = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  // using GoalHandleNavigateToPose_c_ = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MyNode(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name)
  {
    // se subscribe al mapa
    sub_map_global_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10, std::bind(&MyNode::callback_map_global, this, _1));

    sub_map_robot_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "my_map", 10, std::bind(&MyNode::callback_map_robot, this, _1));

    // publica visual markers
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("my_markers", 10);
    timer_markers_ = create_wall_timer(
      rate, std::bind(&MyNode::timer_callback_markers, this));

    // publica poses
    pub_poses_ = create_publisher<geometry_msgs::msg::PoseArray>("my_poses", 10);
    timer_poses_ = create_wall_timer(
      rate, std::bind(&MyNode::timer_callback_poses, this));

  }


  void timer_callback_markers()
  {
    visualization_msgs::msg::MarkerArray total_markers_unknown_limit_;

    for (size_t i = 0; i < pose_array_unknown_limit_.poses.size(); i++) {
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.header.frame_id = "map";
      // marker.header.stamp = ros::Time::now();
      marker.id = i;
      marker.lifetime.sec = 2;
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

    pub_markers_->publish(total_markers_unknown_limit_);

  }

  void timer_callback_poses()
  {
    pose_array_unknown_limit_.poses.clear();
    // if (pose_array_unknown_limit_global_.poses.size() && pose_array_unknown_limit_robot_.poses.size()) {
    //   RCLCPP_INFO(this->get_logger(), "x global %f x robot %f ", pose_array_unknown_limit_global_.poses[0].position.x, pose_array_unknown_limit_robot_.poses[0].position.x);
    //   RCLCPP_INFO(this->get_logger(), "y global %f y robot %f ", pose_array_unknown_limit_global_.poses[0].position.y, pose_array_unknown_limit_robot_.poses[0].position.y);
    //   // RCLCPP_INFO(this->get_logger(), "x robot %f x global %f ", pose_array_unknown_limit_global_.poses[1].position.x, pose_array_unknown_limit_robot_.poses[1].position.x);
    //   // RCLCPP_INFO(this->get_logger(), "y robot %f y global %f ", pose_array_unknown_limit_global_.poses[1].position.y, pose_array_unknown_limit_robot_.poses[1].position.y);
    //   // para quedarnos solo con tres decimales:
    //   float x_robot = pose_array_unknown_limit_robot_.poses[0].position.x * 1000;
    //   x_robot = ceil(x_robot);
    //   x_robot = x_robot / 1000;
    //   RCLCPP_INFO(this->get_logger(), "x robot 3 decimales %f ", x_robot);

    //   float x_global = pose_array_unknown_limit_global_.poses[0].position.x * 1000;
    //   x_global = ceil(x_global);
    //   x_global = x_global / 1000;
    //   RCLCPP_INFO(this->get_logger(), "x global 3 decimales %f ", x_global);
    // }
    // RCLCPP_INFO(this->get_logger(), "x robot %d x global %d ", pose_array_unknown_limit_global_.poses[1].position.x, pose_array_unknown_limit_robot_.poses[1].position.x);
    for (size_t i = 0; i < pose_array_unknown_limit_global_.poses.size(); i++) {
      float x_global = pose_array_unknown_limit_global_.poses[i].position.x * 10;
      x_global = ceil(x_global);
      x_global = x_global / 10;
      float y_global = pose_array_unknown_limit_global_.poses[i].position.y * 10;
      y_global = ceil(y_global);
      y_global = y_global / 10;
      for (size_t l = 0; l < pose_array_unknown_limit_robot_.poses.size(); l++) {
        float x_robot = pose_array_unknown_limit_robot_.poses[l].position.x * 10;
        x_robot = ceil(x_robot);
        x_robot = x_robot / 10;
        float y_robot = pose_array_unknown_limit_robot_.poses[l].position.y * 10;
        y_robot = ceil(y_robot);
        y_robot = y_robot / 10;
        // RCLCPP_INFO(this->get_logger(), "x robot %d x global %d ", pose_array_unknown_limit_global_.poses[l].position.x, pose_array_unknown_limit_robot_.poses[i].position.x);
        if(x_global == x_robot) {
          // RCLCPP_INFO(this->get_logger(), "same x");
          if(y_global == y_robot) {
            // RCLCPP_INFO(this->get_logger(), "same y");
            pose_array_unknown_limit_.poses.push_back(pose_array_unknown_limit_robot_.poses[i]);
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "pose_array_unknown_limit_robot_ %ld pose_array_unknown_limit_global_ %ld pose_array_unknown_limit_ %ld", pose_array_unknown_limit_robot_.poses.size(), pose_array_unknown_limit_global_.poses.size(), pose_array_unknown_limit_.poses.size());
    pub_poses_->publish(pose_array_unknown_limit_);
  }


  bool check_if_limit(nav2_costmap_2d::Costmap2D mapa_, float x, float y, int map_width, int map_height)
  {
    // comprobamos que no vamos a ver un dato fuera del mapa
    if (x + 1 < map_width) {
      char char_value = mapa_.getCost(x + 1, y);
      int value_left = int(char_value);
      if (value_left == 0) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos
        return true;
      }
    }
    if (x - 1 > -1) {
      char char_value = mapa_.getCost(x - 1, y);
      int value_right = int(char_value);
      if (value_right == 0) {
        return true;
      }
    }
    if (y + 1 < map_height) {
      char char_value = mapa_.getCost(x, y + 1);
      int value_up = int(char_value);
      if (value_up == 0) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos
        return true;
      }
    }
    if (y - 1 > -1) {
      char char_value = mapa_.getCost(x, y - 1);
      int value_down = int(char_value);
      if (value_down == 0) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos
        return true;
      }
    }
    return false;
  }

  void occupancygrid_to_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, nav2_costmap_2d::Costmap2D &mapa_)
  {
    float map_resolution = msg->info.resolution;  // resolution [m/cell]
    int map_width = msg->info.width;    // ancho
    int map_height = msg->info.height;  // alto
    float origin_x = msg->info.origin.position.x;
    float origin_y = msg->info.origin.position.y;

    mapa_.resizeMap(map_width, map_height, map_resolution, origin_x, origin_y);

    for (int x = 0; x < map_width; x++) {
      for (int y = 0; y < map_height; y++) {
        int value = msg->data[map_width * (map_height - y - 1) + x];
        mapa_.setCost(x, y, value);
      }
    }
  }

private:

  void callback_map_global(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {

    pose_array_unknown_limit_global_.poses.clear();

    occupancygrid_to_costmap(msg, mapa_costmap_global_);
    
    int map_width = mapa_costmap_global_.getSizeInCellsX();    // ancho
    int map_height = mapa_costmap_global_.getSizeInCellsY();  // alto
    // RCLCPP_INFO(this->get_logger(), "MAP WIDTH %d MAP HEIGHT %d", map_width, map_height);
    for (int x = 0; x < map_width; x++) {
      for (int y = 0; y < map_height; y++) {

        geometry_msgs::msg::Pose pose_;

        double mux = 0;
        double muy = 0;
        mapa_costmap_global_.mapToWorld(x, y, mux, muy);
        pose_.position.x = mux;
        pose_.position.y = -muy;
        // if (x == 0 && y == 0) {
        //   RCLCPP_INFO(this->get_logger(), "map global asigno x robot %f x global %f ", mux, muy);
        // }
        pose_.position.z = 0.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 1.0;

        char char_value = mapa_costmap_global_.getCost(x, y); // me lo devuelve en unsigned char y lo paso a char para poder tener -1
        int value = int(char_value);

        if (value == -1) {
          if (check_if_limit(mapa_costmap_global_, x, y, map_width, map_height)) {
            pose_array_unknown_limit_global_.poses.push_back(pose_);
          }
        }
      }
    }
  }

  void callback_map_robot(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // std::cout << "callback_map_robot" << std::endl;
    pose_array_unknown_limit_robot_.poses.clear();

    occupancygrid_to_costmap(msg, mapa_costmap_robot_);
    
    int map_width = mapa_costmap_robot_.getSizeInCellsX();    // ancho
    int map_height = mapa_costmap_robot_.getSizeInCellsY();  // alto
    // RCLCPP_INFO(this->get_logger(), "MAP WIDTH %d MAP HEIGHT %d", map_width, map_height);
    for (int x = 0; x < map_width; x++) {
      for (int y = 0; y < map_height; y++) {

        geometry_msgs::msg::Pose pose_;

        double mux = 0;
        double muy = 0;
        mapa_costmap_robot_.mapToWorld(x, y, mux, muy);
        pose_.position.x = mux;
        pose_.position.y = -muy;
        // if (x == 0 && y == 0) {
        //   RCLCPP_INFO(this->get_logger(), "map robot asigno x robot %f x global %f ", mux, muy);
        // }
        pose_.position.z = 0.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 1.0;

        char char_value = mapa_costmap_robot_.getCost(x, y); // me lo devuelve en unsigned char y lo paso a char para poder tener -1
        int value = int(char_value);

        if (value == -1) {
          if (check_if_limit(mapa_costmap_robot_, x, y, map_width, map_height)) {
            pose_array_unknown_limit_robot_.poses.push_back(pose_);
          }
        }
      }
    }
  }


  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_global_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_robot_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_markers_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;
  rclcpp::TimerBase::SharedPtr timer_poses_;

  geometry_msgs::msg::PoseArray pose_array_unknown_limit_global_;
  geometry_msgs::msg::PoseArray pose_array_unknown_limit_robot_;

  geometry_msgs::msg::PoseArray pose_array_unknown_limit_;

  nav2_costmap_2d::Costmap2D mapa_costmap_global_;
  nav2_costmap_2d::Costmap2D mapa_costmap_robot_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // geometry_msgs::msg::PoseArray pose_array_;

  auto node_A = std::make_shared<MyNode>("node_A", 1s);

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node_A);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
