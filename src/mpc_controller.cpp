// Copyright (c) 2021 Harderthan
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

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_mpc_controller/mpc_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

// using std::hypot;
// using std::min;
// using std::max;
// using std::abs;
// using nav2_util::declare_parameter_if_not_declared;
// using nav2_util::geometry_utils::euclidean_distance;
// using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_mpc_controller
{
  void MPCController::configure(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();

    double transform_tolerance = 0.1;
    double control_frequency = 20.0;

    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_ = 1.0 / control_frequency;

    // if (inflation_cost_scaling_factor_ <= 0.0)
    // {
    //   RCLCPP_WARN(logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
    //                        "it should be >0. Disabling cost regulated linear velocity scaling.");
    //   use_cost_regulated_linear_velocity_scaling_ = false;
    // }

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    // carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
    // carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
  }

  void MPCController::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type"
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_.reset();
    // carrot_pub_.reset();
    // carrot_arc_pub_.reset();
  }

  void MPCController::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type "
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_->on_activate();
    // carrot_pub_->on_activate();
    // carrot_arc_pub_->on_activate();
  }

  void MPCController::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type "
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    // carrot_pub_->on_deactivate();
    // carrot_arc_pub_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &speed)
  {

    double linear_vel = 0.5;
    double angular_vel = 0.5;

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = linear_vel;
    cmd_vel.twist.angular.z = angular_vel;
    return cmd_vel;
  }

  void MPCController::setPlan(const nav_msgs::msg::Path &path)
  {
    global_plan_ = path;
  }
} // namespace nav2_mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
    nav2_mpc_controller::MPCController,
    nav2_core::Controller)
