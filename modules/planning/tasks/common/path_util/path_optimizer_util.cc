/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "modules/planning/tasks/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

FrenetFramePath PathOptimizerUtil::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

double PathOptimizerUtil::EstimateJerkBoundary(const double vehicle_speed,
                                               const double axis_distance,
                                               const double max_yaw_rate) {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

std::vector<common::PathPoint>
PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
      apollo::common::VehicleConfigHelper::GetConfig()
          .vehicle_param()
          .wheel_base();
  for (auto path_point : path_data.discretized_path()) {
    common::PathPoint new_path_point = path_point;
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));
    ret.push_back(new_path_point);
  }
  return ret;
}

bool PathOptimizerUtil::OptimizePath(
    const SLState& init_state, const std::array<double, 3>& end_state,
    std::vector<double> l_ref, std::vector<double> l_ref_weight,
    const PathBoundary& path_boundary,
    const std::vector<std::pair<double, double>>& ddl_bounds, double dddl_bound,
    const PiecewiseJerkPathConfig& config, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  // num of knots
  const auto& lat_boundaries = path_boundary.boundary();
  const size_t kNumKnots = lat_boundaries.size();

  double delta_s = path_boundary.delta_s();
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);

  // TODO(Hongyi): update end_state settings
  std::array<double, 3U> end_state_weight = {config.weight_end_state_l(),
                                             config.weight_end_state_dl(),
                                             config.weight_end_state_ddl()};
  piecewise_jerk_problem.set_end_state_ref(end_state_weight, end_state);
  piecewise_jerk_problem.set_x_ref(std::move(l_ref_weight), l_ref);
  // for debug:here should use std::move
  piecewise_jerk_problem.set_weight_x(config.l_weight());
  piecewise_jerk_problem.set_weight_dx(config.dl_weight());
  piecewise_jerk_problem.set_weight_ddx(config.ddl_weight());
  piecewise_jerk_problem.set_weight_dddx(config.dddl_weight());

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(
      -config.lateral_derivative_bound_default(),
      config.lateral_derivative_bound_default());
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  piecewise_jerk_problem.set_dddx_bound(dddl_bound);

  bool success = piecewise_jerk_problem.Optimize(config.max_iteration());

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    AINFO << "INIT s(" << init_state.first[0] << "," << init_state.first[1]
          << "," << init_state.first[2] << ") l (" << init_state.second[0]
          << "," << init_state.second[1] << "," << init_state.second[2];
    AINFO << "jerk bound" << dddl_bound;
    std::stringstream ssm;
    for (size_t i = 0; i < lat_boundaries.size(); i++) {
      ssm << lat_boundaries[i].first << " " << lat_boundaries[i].second << ","
          << ddl_bounds[i].first << " " << ddl_bounds[i].second << ","
          << l_ref[i] << std::endl;
    }
    AERROR << "lat boundary, ddl boundary , path reference" << std::endl
           << ssm.str();
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

void PathOptimizerUtil::UpdatePathRefWithBound(
    const PathBoundary& path_boundary, std::vector<double>& ref_l,
    std::vector<double>& weight_ref_l, double weight) {
  auto& boundary = path_boundary.boundary();
  CHECK_EQ(boundary.size(), ref_l.size());
  CHECK_EQ(boundary.size(), weight_ref_l.size());
  for (size_t i = 0; i < ref_l.size(); i++) {
    if (ref_l[i] < boundary[i].first || ref_l[i] > boundary[i].second) {
      ref_l[i] = (boundary[i].first + boundary[i].second) / 2.0;
      weight_ref_l[i] = weight;
    } else {
      weight_ref_l[i] = 0;
    }
  }
}

}  // namespace planning
}  // namespace apollo
