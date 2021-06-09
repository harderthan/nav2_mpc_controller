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

using nav2_util::declare_parameter_if_not_declared;

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

    declare_parameter_if_not_declared(
        node, plugin_name_ + ".delay_mode", rclcpp::ParameterValue(false));
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

    node->get_parameter(plugin_name_ + ".delay_mode", delay_mode_);
    node->get_parameter(plugin_name_ + ".thread_numbers", thread_numbers_);
    node->get_parameter(plugin_name_ + ".max_speed", max_speed_);
    node->get_parameter(plugin_name_ + ".waypoints_dist", waypointsDist_);
    node->get_parameter(plugin_name_ + ".pathLength", pathLength_);
    node->get_parameter(plugin_name_ + ".goal_radius", goalRadius_);
    node->get_parameter(plugin_name_ + ".controller_freq", controller_freq_);

    throttle_ = 0.0;
    w_ = 0.0;
    speed_ = 0.0;

    dt_ = double(1.0 / controller_freq_); // time step duration dt in s
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

    //Init parameters for MPC object
    _mpc_params["DT"] = dt_;
    _mpc_params["STEPS"] = mpc_steps_;
    _mpc_params["REF_CTE"] = ref_cte_;
    _mpc_params["REF_ETHETA"] = ref_etheta_;
    _mpc_params["REF_V"] = ref_vel_;
    _mpc_params["W_CTE"] = w_cte_;
    _mpc_params["W_EPSI"] = w_etheta_;
    _mpc_params["W_V"] = w_vel_;
    _mpc_params["W_ANGVEL"] = w_angvel_;
    _mpc_params["W_A"] = w_accel_;
    _mpc_params["W_DANGVEL"] = w_angvel_d_;
    _mpc_params["W_DA"] = w_accel_d_;
    _mpc_params["ANGVEL"] = max_angvel_;
    _mpc_params["MAXTHR"] = max_throttle_;
    _mpc_params["BOUND"] = bound_value_;
    _mpc.LoadParams(_mpc_params);

    double transform_tolerance = 0.1;
    double control_frequency = 20.0;

    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_ = 1.0 / control_frequency;

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  }

  void MPCController::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type"
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_.reset();
  }

  void MPCController::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type "
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_->on_activate();
  }

  void MPCController::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type "
        " nav2_mpc_controller::MPCController",
        plugin_name_.c_str());
    global_path_pub_->on_deactivate();
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
    const double costheta = cos(theta);
    const double sintheta = sin(theta);
    const double v = speed.linear.x;
    const double w = w_;               // steering -> w
    const double throttle = throttle_; // accel: >0; brake: <0
    const double dt = dt_;

    // Waypoints related parameters
    const int N = global_plan_.poses.size();

    // Convert to the vehicle coordinate system
    Eigen::VectorXd x_veh(N);
    Eigen::VectorXd y_veh(N);
    for (int i = 0; i < N; i++)
    {
      const double dx = global_plan_.poses[i].pose.position.x - px;
      const double dy = global_plan_.poses[i].pose.position.y - py;
      x_veh[i] = dx * costheta + dy * sintheta;
      y_veh[i] = dy * costheta - dx * sintheta;
    }

    // Fit waypoints
    auto coeffs = polyfit(x_veh, y_veh, 3);
    const double cte = polyeval(coeffs, 0.0);
    cout << "coeffs : " << coeffs[0] << endl;
    cout << "pow : " << pow(0.0, 0) << endl;
    cout << "cte : " << cte << endl;
    double etheta = atan(coeffs[1]);

    // Global coordinate system about theta
    double gx = 0;
    double gy = 0;
    int N_sample = N * 0.3;
    for (int i = 1; i < N_sample; i++)
    {
      gx += global_plan_.poses[i].pose.position.x - global_plan_.poses[i - 1].pose.position.x;
      gy += global_plan_.poses[i].pose.position.y - global_plan_.poses[i - 1].pose.position.y;
    }

    double temp_theta = theta;
    double traj_deg = atan2(gy, gx);
    double PI = 3.141592;

    // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity
    if (temp_theta <= -PI + traj_deg)
      temp_theta = temp_theta + 2 * PI;

    // Implementation about theta error more precisly
    if (gx && gy && temp_theta - traj_deg < 1.8 * PI)
      etheta = temp_theta - traj_deg;
    else
      etheta = 0;
    cout << "etheta: " << etheta << ", atan2(gy,gx): " << atan2(gy, gx) << ", temp_theta:" << traj_deg << endl;

    Eigen::VectorXd state(6);
    if (delay_mode_)
    {
      // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
      const double px_act = v * dt;
      const double py_act = 0;
      const double theta_act = w * dt;        //(steering) theta_act = v * steering * dt / Lf;
      const double v_act = v + throttle * dt; //v = v + a * dt

      const double cte_act = cte + v * sin(etheta) * dt;
      const double etheta_act = etheta - theta_act;

      state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
    }
    else
    {
      state << 0, 0, 0, v, cte, etheta;
    }

    // Solve MPC Problem
    rclcpp::Time begin = rclcpp::Clock().now();
    vector<double> mpc_results = _mpc.Solve(state, coeffs);
    rclcpp::Time end = rclcpp::Clock().now();
    cout << "Duration: " << end.seconds() << "." << end.nanoseconds() << endl
         << begin.seconds() << "." << begin.nanoseconds() << endl;

    // MPC result (all described in car frame), output = (acceleration, w)
    w_ = mpc_results[0];        // radian/sec, angular velocity
    throttle_ = mpc_results[1]; // acceleration

    speed_ = v + throttle_ * dt_; // speed
    if (speed_ >= max_speed_)
      speed_ = max_speed_;
    if (speed_ <= 0.0)
      speed_ = 0.0;

    if (debug_info_)
    {
      cout << "\n\nDEBUG" << endl;
      cout << "theta: " << theta << endl;
      cout << "V: " << v << endl;
      cout << "coeffs: \n"
           << coeffs << endl;
      cout << "w_: \n"
           << w_ << endl;
      cout << "throttle_: \n"
           << throttle_ << endl;
      cout << "speed_: \n"
           << speed_ << endl;
    }

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = speed_;
    cmd_vel.twist.angular.z = w_;
    return cmd_vel;
  }

  // Evaluate a polynomial.
  double MPCController::polyeval(Eigen::VectorXd coeffs, double x)
  {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  // Fit a polynomial.
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd MPCController::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
  {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
      A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
      for (int i = 0; i < order; i++)
        A(j, i + 1) = A(j, i) * xvals(j);
    }
    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
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
