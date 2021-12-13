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
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <memory>
#include <cmath>
#include <thread>
#include <functional>

using namespace std;

using namespace std::placeholders;

using namespace std::chrono_literals;


class MyNode : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose_s_ = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GoalHandleNavigateToPose_c_ = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MyNode(const std::string & name, const std::chrono::nanoseconds & rate)
  : Node(name)
  {
    // se subscribe al mapa 
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MyNode::callback_map, this, _1));

    // se subscribe a la posicion del robot en el mundo, no en el mapa
    sub_robot_pos_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MyNode::callback_pos, this, _1));
    
    // publica visual markers
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mis_markers", 10);
    timer_markers_ = create_wall_timer(
      rate, std::bind(&MyNode::timer_callback_markers, this));
    
    pub_goal_marker_ = create_publisher<visualization_msgs::msg::Marker>("/mi_marker_goal", 10);
    timer_goal_marker_ = create_wall_timer(
      rate, std::bind(&MyNode::timer_callback_goal_marker, this));

  }

  // cliente
  void start_client() 
  {
    navigate_to_pose_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      shared_from_this(), "navigate_to_pose");
    
    if (!this->navigate_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }  
  }

  void call_server() 
  {
    auto goal_pose = NavigateToPose::Goal();

    goal_pose.pose.pose = goal_pos_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // send_goal_options.goal_response_callback =
    //   std::bind(&MyNode::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&MyNode::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&MyNode::result_callback, this, _1);
    
    auto goal_handle_future = navigate_to_pose_action_client_->async_send_goal(
      goal_pose, send_goal_options);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "send_goal failed");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(
        get_logger(), "ExecutorClient: Execution was rejected by the action server");
      return;
    }
  }

  void timer_callback_markers()
  {
    visualization_msgs::msg::MarkerArray total_markers_unknown_limit_;

    for (int i = 0; i < int(pose_array_unknown_limit_.poses.size()); i++) {
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.header.frame_id = "map";
      // marker.header.stamp = ros::Time::now();
      marker.id = i;
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

    pub_markers_->publish(total_markers_unknown_limit_);

    pose_array_unknown_limit_.poses.clear();

  }

  void timer_callback_goal_marker()
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time::now();
    marker.id = 1 + int(pose_array_unknown_limit_.poses.size());
    marker.lifetime.sec = 5;
    marker.pose.position.x = goal_pos_.position.x;
    marker.pose.position.y = goal_pos_.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha! or your marker will be invisible!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
 
    pub_goal_marker_->publish(marker);

  }

  bool check_if_limit(float x, float y, int map_width, int map_height) 
  {
    // comprobamos que no vamos a ver un dato fuera del mapa 
    if(x + 1 < map_width) {
      char char_value = mapa_costmap_.getCost(x+1, y);
      int value_left = int(char_value);
      if(value_left != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    if(x - 1 > -1) {
      char char_value = mapa_costmap_.getCost(x-1, y);
      int value_right = int(char_value);
      if(value_right != -1) {
        return true;
      }
    }
    if(y + 1 < map_height) {
      char char_value = mapa_costmap_.getCost(x, y+1);
      int value_up = int(char_value);
      if(value_up != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    if(y - 1 > -1) {
      char char_value = mapa_costmap_.getCost(x, y-1);
      int value_down = int(char_value);
      if(value_down != -1) {
        // si uno de nuestros vecinos es distinto de -1 quiere decir que nuestro punto es un limite a los desconocidos 
        return true;
      }
    }
    return false; 
  }

  void occupancygrid_to_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    float map_resolution = msg->info.resolution;  // resolution [m/cell]
    int map_width = msg->info.width;    // ancho
    int map_height = msg->info.height;  // alto
    float origin_x = msg->info.origin.position.x;
    float origin_y = msg->info.origin.position.y;

    mapa_costmap_.resizeMap(map_width, map_height, map_resolution, origin_x, origin_y);

    for(int x = 0; x < map_width; x++) {
      for(int y = 0; y < map_height; y++) {
        int value = msg->data[map_width * (map_height - y - 1) + x];
        mapa_costmap_.setCost(x, y, value);       
      }
    }
  }

  void where_to_go()
  {
    double less_distance_ = -1;

    for (int i = 0; i < int(pose_array_unknown_limit_.poses.size()); i++) {
      float x_ = pose_array_unknown_limit_.poses[i].position.x;
      float y_ = pose_array_unknown_limit_.poses[i].position.y;
      double distance_ = abs(x_ - robot_pos_.position.x) + abs(y_ - robot_pos_.position.y);
      if(less_distance_ == -1 || distance_ < less_distance_) {
        less_distance_ = distance_;
        goal_pos_.position.x = x_;
        goal_pos_.position.y = y_;
      }

    }
    RCLCPP_INFO(this->get_logger(), "punto deseado: %f %f", goal_pos_.position.x, goal_pos_.position.y);
  }

private:
  void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
    
    pose_array_unknown_.poses.clear();
    pose_array_unknown_limit_.poses.clear();

    occupancygrid_to_costmap(msg);

    int map_width = mapa_costmap_.getSizeInCellsX();    // ancho
    int map_height = mapa_costmap_.getSizeInCellsY();  // alto
    
    for(int x = 0; x < map_width; x++) {
      for(int y = 0; y < map_height; y++) {

        geometry_msgs::msg::Pose pose_;
 
        double mux = 0;
        double muy = 0;
        mapa_costmap_.mapToWorld(x, y, mux, muy);
        pose_.position.x = mux;
        pose_.position.y = -muy;
        pose_.position.z = 0.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 1.0;

        char char_value = mapa_costmap_.getCost(x, y); // me lo devuelve en unsigned char y lo paso a char para poder tener -1
        int value = int(char_value);

        if(value == -1) {
          if (check_if_limit(x, y, map_width, map_height)) {
            pose_array_unknown_limit_.poses.push_back(pose_);
          } else {
            pose_array_unknown_.poses.push_back(pose_);
          }       
        }
      }
    }
    where_to_go();

    // RCLCPP_INFO(this->get_logger(), "llamamos al servidor");
    // call_server();

    
  }

  void callback_pos(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {
    robot_pos_.position.x = msg->pose.pose.position.x;
    robot_pos_.position.y = msg->pose.pose.position.y;
    robot_pos_.position.z = msg->pose.pose.position.z;
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_pos_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_markers_;
  rclcpp::TimerBase::SharedPtr timer_goal_marker_;
  geometry_msgs::msg::PoseArray pose_array_unknown_;
  geometry_msgs::msg::PoseArray pose_array_unknown_limit_;

  nav2_costmap_2d::Costmap2D mapa_costmap_;

  geometry_msgs::msg::Pose robot_pos_;
  geometry_msgs::msg::Pose goal_pos_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_action_client_;

  // A callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    // This implementation just accepts all goals.
    RCLCPP_INFO(this->get_logger(), "Received goal request with pose [%f, %f, %f]", goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // if (goal->times > 0) {
    //   current_goal_ = *goal;
    //   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // } else {
    //   return rclcpp_action::GoalResponse::REJECT;
    // }
  }

  // void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
  // {
  //   auto goal_handle = future.get();
  //   if (!goal_handle) {
  //     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  //   }
  // }

  void feedback_callback(GoalHandleNavigateToPose_c_::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "queda una distancia de %f para llegar al objetivo", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose_c_::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  geometry_msgs::msg::PoseArray pose_array_;

  auto node_A = std::make_shared<MyNode>("node_A", 1s);

  node_A->start_client();
  node_A->call_server();

  rclcpp::executors::SingleThreadedExecutor executor;
  
  executor.add_node(node_A);
  
  executor.spin();

  rclcpp::shutdown();

  return 0;
}