/*
# Copyright 2021 Harderthan
# Original Code is from https://github.com/Geonhee-LEE/mpc_ros
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#ifndef MPC_H
#define MPC_H

#include <Eigen/Core>
#include <map>
#include <vector>

using namespace std;

class MPCCore {
public:
  MPCCore();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
  vector<double> mpc_theta;

  void LoadParams(const std::map<string, double> &params);

private:
  // Parameters for mpc solver
  double _max_angvel, _max_throttle, _bound_value;
  int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start,
      _etheta_start, _angvel_start, _a_start;
  std::map<string, double> _params;

  unsigned int dis_cnt;
};

#endif /* MPC_H */
