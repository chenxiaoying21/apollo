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

#include "modules/planning/planning_base/task_base/common/path_util/path_bounds_decider_util.h"
#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;

bool PathBoundsDeciderUtil::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound,
    SLState init_sl_state) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();

  for (double curr_s = init_sl_state.first[0];
       curr_s < std::fmin(init_sl_state.first[0] +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += FLAGS_path_bounds_decider_resolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

void PathBoundsDeciderUtil::GetStartPoint(
    common::TrajectoryPoint planning_start_point,
    const ReferenceLine& reference_line, SLState* init_sl_state) {
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  *init_sl_state = reference_line.ToFrenetFrame(planning_start_point);
}

double PathBoundsDeciderUtil::GetADCLaneWidth(
    const ReferenceLine& reference_line, const double adc_s) {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_s, &lane_left_width,
                                   &lane_right_width)) {
    constexpr double kDefaultLaneWidth = 5.0;
    AWARN << "Failed to get lane width at planning start point.";
    return kDefaultLaneWidth;
  } else {
    return lane_left_width + lane_right_width;
  }
}

bool PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries) {
  // substract vehicle width when bound does not come from the lane boundary
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

void PathBoundsDeciderUtil::TrimPathBounds(const int path_blocked_idx,
                                           PathBound* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

bool PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    PathBound* const path_boundaries, std::string* const blocking_obstacle_id) {
  // Preprocessing.
  auto indexed_obstacles = reference_line_info.path_decision().obstacles();
  auto sorted_obstacles =
      SortObstaclesForSweepLine(indexed_obstacles, init_sl_state);
  ADEBUG << "There are " << sorted_obstacles.size() << " obstacles.";
  double center_line = init_sl_state.second[0];
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
               << "] curr_obstacle_l_min[" << curr_obstacle_l_min
               << "] curr_obstacle_l_max[" << curr_obstacle_l_max
               << "] center_line[" << center_line << "]";
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
          }
          if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                  i, *left_bounds.begin(), *right_bounds.begin(),
                  path_boundaries, &center_line)) {
            path_blocked_idx = static_cast<int>(i);
            *blocking_obstacle_id = curr_obstacle_id;
            break;
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<ObstacleEdge> PathBoundsDeciderUtil::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles,
    const SLState& init_sl_state) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obstacle->PerceptionSLBoundary().end_s() < init_sl_state.first[0]) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

bool PathBoundsDeciderUtil::UpdatePathBoundaryAndCenterLineWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  UpdatePathBoundaryWithBuffer(idx, left_bound, right_bound, path_boundaries);
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

double PathBoundsDeciderUtil::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  static constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(
    const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() ||
      obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }
  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

bool PathBoundsDeciderUtil::ComputeSLBoundaryIntersection(
    const SLBoundary& sl_boundary, const double s, double* ptr_l_min,
    double* ptr_l_max) {
  *ptr_l_min = std::numeric_limits<double>::max();
  *ptr_l_max = -std::numeric_limits<double>::max();

  // invalid polygon
  if (sl_boundary.boundary_point_size() < 3) {
    return false;
  }

  bool has_intersection = false;
  for (auto i = 0; i < sl_boundary.boundary_point_size(); ++i) {
    auto j = (i + 1) % sl_boundary.boundary_point_size();
    const auto& p0 = sl_boundary.boundary_point(i);
    const auto& p1 = sl_boundary.boundary_point(j);

    if (common::util::WithinBound<double>(std::fmin(p0.s(), p1.s()),
                                          std::fmax(p0.s(), p1.s()), s)) {
      has_intersection = true;
      auto l = common::math::lerp<double>(p0.l(), p0.s(), p1.l(), p1.s(), s);
      if (l < *ptr_l_min) {
        *ptr_l_min = l;
      }
      if (l > *ptr_l_max) {
        *ptr_l_max = l;
      }
    }
  }
  return has_intersection;
}

common::TrajectoryPoint
PathBoundsDeciderUtil::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

bool PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    const bool is_extend_adc_bound, const double extend_buffer,
    PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double adc_lane_width =
      GetADCLaneWidth(reference_line, init_sl_state.first[0]);
  double offset_to_map = 0;
  reference_line.GetOffsetToMap(init_sl_state.first[0], &offset_to_map);
  double adc_l_to_lane_center = init_sl_state.second[0] + offset_to_map;
  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double ADC_speed_buffer =
        (init_sl_state.second[1] > 0 ? 1.0 : -1.0) * init_sl_state.second[1] *
        init_sl_state.second[1] / kMaxLateralAccelerations / 2.0;

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    double adc_half_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    if (is_extend_adc_bound) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center,
                    adc_l_to_lane_center + ADC_speed_buffer) +
          adc_half_width + extend_buffer;
      curr_left_bound =
          std::fmax(curr_lane_left_width, curr_left_bound_adc) - offset_to_map;

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center,
                    adc_l_to_lane_center + ADC_speed_buffer) -
          adc_half_width - extend_buffer;
      curr_right_bound =
          std::fmin(-curr_lane_right_width, curr_right_bound_adc) -
          offset_to_map;
    } else {
      curr_left_bound = curr_lane_left_width - offset_to_map;
      curr_right_bound = -curr_lane_right_width - offset_to_map;
    }

    ADEBUG << "At s = " << curr_s
           << ", left_lane_bound = " << curr_lane_left_width
           << ", right_lane_bound = " << curr_lane_right_width
           << ", offset = " << offset_to_map;

    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);

  return true;
}

void PathBoundsDeciderUtil::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    std::get<1>((*path_bound)[i]) -= refline_offset_to_lane_center;
    std::get<2>((*path_bound)[i]) -= refline_offset_to_lane_center;
  }
}

int PathBoundsDeciderUtil::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y,
    const std::vector<std::tuple<double, double, double>>& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  if (point_sl.s() > std::get<0>(path_bound.back()) ||
      point_sl.s() < std::get<0>(path_bound.front()) -
                         FLAGS_path_bounds_decider_resolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         std::get<0>(path_bound[idx_after]) < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << std::get<1>(path_bound[idx_after]) << ", "
         << std::get<2>(path_bound[idx_after]) << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (std::get<1>(path_bound[idx_before]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_before]) >= point_sl.l() &&
      std::get<1>(path_bound[idx_after]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_after]) >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

}  // namespace planning
}  // namespace apollo
