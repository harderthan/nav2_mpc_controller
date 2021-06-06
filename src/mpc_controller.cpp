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

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// using std::hypot;
// using std::min;
// using std::max;
// using std::abs;
using nav2_util::declare_parameter_if_not_declared;
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

    throttle_ = 0.0; 
    w_ = 0.0;
    speed_ = 0.0;

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".thread_numbers", rclcpp::ParameterValue(2));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".waypoints_dist", rclcpp::ParameterValue(-1.0));
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".pathLength", rclcpp::ParameterValue(8.0)); // unit: m
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".goal_radius", rclcpp::ParameterValue(0.5)); // unit: m
    declare_parameter_if_not_declared(
        node, plugin_name_ + ".controller_freq", rclcpp::ParameterValue(10)); 
    // declare_parameter_if_not_declared(
    //     node, plugin_name_ + ".vehicle_Lf", rclcpp::ParameterValue(0.290)); 

    node->get_parameter(plugin_name_ + ".thread_numbers", thread_numbers_);
    node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
    node->get_parameter(plugin_name_ + ".waypoints_dist", waypointsDist_);
    node->get_parameter(plugin_name_ + ".pathLength", pathLength_);
    node->get_parameter(plugin_name_ + ".goal_radius", goalRadius_);
    node->get_parameter(plugin_name_ + ".controller_freq", controller_freq_);
    // node->get_parameter(plugin_name_ + ".vehicle_Lf", Lf_);

    dt_ = double(1.0/controller_freq_); // time step duration dt in s
    mpc_steps_ = 20.0;
    ref_cte_ = 0.0;
    ref_vel_ = 1.0;
    ref_etheta_ = 0.0;
    w_cte_ = 5000.0;
    w_etheta_ = 5000.0;
    w_vel_ = 1.0;
    w_angvel_ = 100.0;
    w_angvel_d_ = 10.0;
    w_accel_ = 50.0;
    w_accel_d_ = 10.0;
    max_angvel_ = 3.0; // Maximal angvel radian (~30 deg)
    max_throttle_ = 1.0;
    bound_value_ = 1.0e3;

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
    // Update system states: X=[x, y, theta, v]
    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    const double theta = tf2::getYaw(q);
    const double v = speed.linear.x;
    // Update system inputs: U=[w, throttle]
    RCLCPP_INFO(logger_, "computeVelocityCommand theta: %f, linear_v: %f", theta, v);

    // const double w = _w; // steering -> w
    // //const double steering = _steering;  // radian
    // const double throttle = _throttle; // accel: >0; brake: <0
    // const double dt = _dt;
    // //const double Lf = _Lf;

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
