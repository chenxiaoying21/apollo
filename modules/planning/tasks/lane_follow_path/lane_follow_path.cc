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

#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <memory>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/tasks/common/path_generation.h"
#include "modules/planning/tasks/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/tasks/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/tasks/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool LaneFollowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

apollo::common::Status LaneFollowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time path empty:"
           << reference_line_info->path_data().Empty()
           << "path reusable: " << reference_line_info->path_reusable();
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!PathBoundsDecider(candidate_path_boundaries)) {
    return Status::OK();
  }
  if (!PathOptimizer(candidate_path_boundaries, candidate_path_data)) {
    return Status::OK();
  }
  if (!PathAssessmentDecider(candidate_path_data,
                             reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();
}

bool LaneFollowPath::PathBoundsDecider(std::vector<PathBoundary>& boundary) {
  PathBound path_bound;
  std::string blocking_obstacle_id = "";
  std::string lane_type = "";
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  std::string borrow_lane_type;
  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_,
          config_.is_extend_lane_bounds_to_include_adc(),
          config_.extend_buffer(), &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on lane and adc.";
    return false;
  }
  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = path_bound;
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, init_sl_state_, &path_bound,
          &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return false;
  }
  // 4. Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }
  ADEBUG << "Completed generating path boundaries.";
  if (config_.is_extend_lane_bounds_to_include_adc()) {
    CHECK_LE(init_sl_state_.second[0], std::get<2>(path_bound[0]));
    CHECK_GE(init_sl_state_.second[0], std::get<1>(path_bound[0]));
  }

  // std::vector<std::pair<double, double>> regular_path_bound_pair;
  // for (size_t i = 0; i < path_bound.size(); ++i) {
  //   regular_path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
  //                                        std::get<2>(path_bound[i]));
  // }
  boundary.emplace_back(FLAGS_path_bounds_decider_resolution, path_bound);
  boundary.back().set_label(absl::StrCat("regular/", "self"));
  boundary.back().set_blocking_obstacle_id(blocking_obstacle_id);
  RecordDebugInfo(path_bound, boundary.back().label(), reference_line_info_);
  return true;
}

bool LaneFollowPath::PathOptimizer(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>& candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double lat_acc_bound =
      std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
      veh_param.wheel_base();
  const double axis_distance = veh_param.wheel_base();
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    CHECK_GT(path_boundary_size, 1U);
    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1.0), axis_distance, max_yaw_rate);
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size,
                                     config.path_reference_l_weight());
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      for (size_t i = 0; i < path_boundary_size; i += 4) {
        ADEBUG << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               << "], l = " << opt_l[i] << ", dl = " << opt_dl[i];
      }
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
    return false;
  }
  return true;
}

bool LaneFollowPath::PathAssessmentDecider(
    std::vector<PathData>& candidate_path_data, PathData* final_path) {
  PathData& curr_path_data = candidate_path_data.back();
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                     curr_path_data)) {
    AINFO << "Lane follow path is invalid";
    return false;
  }

  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
  curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }
  *final_path = curr_path_data;
  reference_line_info_->SetBlockingObstacle(
      curr_path_data.blocking_obstacle_id());
  return true;
}

}  // namespace planning
}  // namespace apollo
