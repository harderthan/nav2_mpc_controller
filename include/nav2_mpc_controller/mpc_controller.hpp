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

#ifndef NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mpc_core.hpp"
#include <Eigen/Core>
#include <Eigen/QR>

namespace nav2_mpc_controller {
/**
 * @class nav2_mpc_controller::MPCController
 * @brief mpc controller plugin
 */
class MPCController : public nav2_core::Controller {
public:
  /**
   * @brief Constructor for nav2_mpc_controller::MPCController
   */
  MPCController() = default;

  /**
   * @brief Destrructor for nav2_mpc_controller::MPCController
   */
  ~MPCController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &parent,
                 std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                     &costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not NULL, will be filled in with full
   * evaluation results
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path &path) override;

protected:
  double impThetaError(double theta, const Eigen::VectorXd coeffs, int sample_size,
                       int sample_ratio);

  double polyeval(Eigen::VectorXd coeffs, double x);
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};

  MPCCore _mpc;
  map<string, double> _mpc_params;
  double mpc_steps_, ref_cte_, ref_etheta_, ref_vel_, w_cte_, w_etheta_, w_vel_,
      w_angvel_, w_accel_, w_angvel_d_, w_accel_d_, max_angvel_, max_throttle_,
      bound_value_;

  double dt_, w_, throttle_, speed_, max_speed_;
  double pathLength_, goalRadius_, waypointsDist_;
  int controller_freq_, downSampling_, thread_numbers_;
  bool goal_received_, goal_reached_, path_computed_, pub_twist_flag_,
      debug_info_, delay_mode_;

  tf2::Duration transform_tolerance_;
  double control_duration_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double inflation_cost_scaling_factor_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      global_path_pub_;
  std::shared_ptr<
      rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
      carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
      carrot_arc_pub_;
};
} // namespace nav2_mpc_controller

#endif // NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
