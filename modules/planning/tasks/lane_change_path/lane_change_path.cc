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

#include "modules/planning/tasks/lane_change_path/lane_change_path.h"

#include <memory>
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/common/path_generation.h"
#include "modules/planning/tasks/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/tasks/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/tasks/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::cyber::Clock;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

bool LaneChangePath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneChangePathConfig>(&config_);
}

apollo::common::Status LaneChangePath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  UpdateLaneChangeStatus();
  const auto& status = injector_->planning_context()
                           ->mutable_planning_status()
                           ->mutable_change_lane()
                           ->status();
  if (!reference_line_info->IsChangeLanePath() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time" << reference_line_info->IsChangeLanePath()
           << "path reusable" << reference_line_info->path_reusable();
    return Status::OK();
  }
  if (status != ChangeLaneStatus::IN_CHANGE_LANE) {
    ADEBUG << injector_->planning_context()
                  ->mutable_planning_status()
                  ->mutable_change_lane()
                  ->DebugString();
    return Status(ErrorCode::PLANNING_ERROR,
                  "Not satisfy lane change  conditions");
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!PathBoundsDecider(candidate_path_boundaries)) {
    return Status(ErrorCode::PLANNING_ERROR, "lane change path bounds failed");
  }
  if (!PathOptimizer(candidate_path_boundaries, candidate_path_data)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "lane change path optimize failed");
  }
  if (!PathAssessmentDecider(candidate_path_data,
                             reference_line_info->mutable_path_data())) {
    return Status(ErrorCode::PLANNING_ERROR, "No valid lane change path");
  }

  return Status::OK();
}

bool LaneChangePath::PathBoundsDecider(std::vector<PathBoundary>& boundary) {
  std::vector<std::tuple<double, double, double>> path_bound;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return false;
  }
  // 2. Decide a rough boundary based on lane info and ADC's position
  PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
      *reference_line_info_, init_sl_state_, true, config_.extend_adc_buffer(),
      &path_bound);

  // 3. Remove the S-length of target lane out of the path-bound.
  GetBoundaryFromLaneChangeForbiddenZone(&path_bound);

  PathBound temp_path_bound = path_bound;
  std::string blocking_obstacle_id;
  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, init_sl_state_, &path_bound,
          &blocking_obstacle_id)) {
    AERROR << "Failed to decide fine tune the boundaries after "
              "taking into consideration all static obstacles.";
    return false;
  }

  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }
  boundary.emplace_back(FLAGS_path_bounds_decider_resolution, path_bound);
  boundary.back().set_label("regular/pullover");
  boundary.back().set_blocking_obstacle_id(blocking_obstacle_id);
  RecordDebugInfo(path_bound, boundary.back().label(), reference_line_info_);
  return true;
}
bool LaneChangePath::PathOptimizer(
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
    std::vector<double> weight_ref_l(path_boundary_size, 0);

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

bool LaneChangePath::PathAssessmentDecider(
    std::vector<PathData>& candidate_path_data, PathData* final_path) {
  std::vector<PathData> valid_path_data;
  for (auto& curr_path_data : candidate_path_data) {
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      SetPathInfo(&curr_path_data);
      PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      if (curr_path_data.Empty()) {
        AINFO << "lane change path is empty after trimed";
        continue;
      }
      valid_path_data.push_back(curr_path_data);
    }
  }
  if (valid_path_data.empty()) {
    AINFO << "All lane change path are not valid";
    return false;
  }

  *final_path = valid_path_data[0];
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

void LaneChangePath::UpdateLaneChangeStatus() {
  std::string change_lane_id;
  auto* prev_status = injector_->planning_context()
                          ->mutable_planning_status()
                          ->mutable_change_lane();
  double now = Clock::NowInSeconds();
  // Init lane change status
  if (!prev_status->has_status()) {
    UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED, "");
    prev_status->set_last_succeed_timestamp(now);
    return;
  }
  bool has_change_lane = frame_->reference_line_info().size() > 1;
  if (!has_change_lane) {
    if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                   prev_status->path_id());
    }
    return;
  }
  // has change lane
  if (reference_line_info_->IsChangeLanePath()) {
    const auto* history_frame = injector_->frame_history()->Latest();
    if (!CheckLastFrameSucceed(history_frame)) {
      UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FAILED, change_lane_id);
      is_exist_lane_change_start_position_ = false;
      return;
    }
    is_clear_to_change_lane_ = IsClearToChangeLane(reference_line_info_);
    change_lane_id = reference_line_info_->Lanes().Id();
    ADEBUG << "change_lane_id" << change_lane_id;
    if (prev_status->status() == ChangeLaneStatus::CHANGE_LANE_FAILED) {
      // TODO(SHU): add an optimization_failure counter to enter
      // change_lane_failed status
      if (now - prev_status->timestamp() >
          config_.change_lane_fail_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        ADEBUG << "change lane again after failed";
      }
      return;
    } else if (prev_status->status() ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      if (now - prev_status->timestamp() >
          config_.change_lane_success_freeze_time()) {
        UpdateStatus(now, ChangeLaneStatus::IN_CHANGE_LANE, change_lane_id);
        AINFO << "change lane again after success";
      }
    } else if (prev_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
      if (prev_status->path_id() != change_lane_id) {
        AINFO << "change_lane_id" << change_lane_id << "prev"
              << prev_status->path_id();
        UpdateStatus(now, ChangeLaneStatus::CHANGE_LANE_FINISHED,
                     prev_status->path_id());
      }
    }
  }
}

bool LaneChangePath::IsClearToChangeLane(
    ReferenceLineInfo* reference_line_info) {
  double ego_start_s = reference_line_info->AdcSlBoundary().start_s();
  double ego_end_s = reference_line_info->AdcSlBoundary().end_s();
  double ego_v =
      std::abs(reference_line_info->vehicle_state().linear_velocity());

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual() || obstacle->IsStatic()) {
      ADEBUG << "skip one virtual or static obstacle";
      continue;
    }

    double start_s = std::numeric_limits<double>::max();
    double end_s = -std::numeric_limits<double>::max();
    double start_l = std::numeric_limits<double>::max();
    double end_l = -std::numeric_limits<double>::max();

    for (const auto& p : obstacle->PerceptionPolygon().points()) {
      apollo::common::SLPoint sl_point;
      reference_line_info->reference_line().XYToSL(p, &sl_point);
      start_s = std::fmin(start_s, sl_point.s());
      end_s = std::fmax(end_s, sl_point.s());

      start_l = std::fmin(start_l, sl_point.l());
      end_l = std::fmax(end_l, sl_point.l());
    }

    if (reference_line_info->IsChangeLanePath()) {
      double left_width(0), right_width(0);
      reference_line_info->mutable_reference_line()->GetLaneWidth(
          (start_s + end_s) * 0.5, &left_width, &right_width);
      if (end_l < -right_width || start_l > left_width) {
        continue;
      }
    }

    // Raw estimation on whether same direction with ADC or not based on
    // prediction trajectory
    bool same_direction = true;
    if (obstacle->HasTrajectory()) {
      double obstacle_moving_direction =
          obstacle->Trajectory().trajectory_point(0).path_point().theta();
      const auto& vehicle_state = reference_line_info->vehicle_state();
      double vehicle_moving_direction = vehicle_state.heading();
      if (vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE) {
        vehicle_moving_direction =
            common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
      }
      double heading_difference = std::abs(common::math::NormalizeAngle(
          obstacle_moving_direction - vehicle_moving_direction));
      same_direction = heading_difference < (M_PI / 2.0);
    }

    // TODO(All) move to confs
    static constexpr double kSafeTimeOnSameDirection = 3.0;
    static constexpr double kSafeTimeOnOppositeDirection = 5.0;
    static constexpr double kForwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kBackwardMinSafeDistanceOnSameDirection = 10.0;
    static constexpr double kForwardMinSafeDistanceOnOppositeDirection = 50.0;
    static constexpr double kBackwardMinSafeDistanceOnOppositeDirection = 1.0;
    static constexpr double kDistanceBuffer = 0.5;

    double kForwardSafeDistance = 0.0;
    double kBackwardSafeDistance = 0.0;
    if (same_direction) {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnSameDirection,
                    (ego_v - obstacle->speed()) * kSafeTimeOnSameDirection);
      kBackwardSafeDistance =
          std::fmax(kBackwardMinSafeDistanceOnSameDirection,
                    (obstacle->speed() - ego_v) * kSafeTimeOnSameDirection);
    } else {
      kForwardSafeDistance =
          std::fmax(kForwardMinSafeDistanceOnOppositeDirection,
                    (ego_v + obstacle->speed()) * kSafeTimeOnOppositeDirection);
      kBackwardSafeDistance = kBackwardMinSafeDistanceOnOppositeDirection;
    }

    if (HysteresisFilter(ego_start_s - end_s, kBackwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking()) &&
        HysteresisFilter(start_s - ego_end_s, kForwardSafeDistance,
                         kDistanceBuffer, obstacle->IsLaneChangeBlocking())) {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(true);
      ADEBUG << "Lane Change is blocked by obstacle" << obstacle->Id();
      return false;
    } else {
      reference_line_info->path_decision()
          ->Find(obstacle->Id())
          ->SetLaneChangeBlocking(false);
    }
  }
  return true;
}

void LaneChangePath::GetLaneChangeStartPoint(
    const ReferenceLine& reference_line, double adc_frenet_s,
    common::math::Vec2d& start_xy) {
  double lane_change_start_s =
      config_.lane_change_prepare_length() + adc_frenet_s;
  common::SLPoint lane_change_start_sl;
  lane_change_start_sl.set_s(lane_change_start_s);
  lane_change_start_sl.set_l(0.0);
  reference_line.SLToXY(lane_change_start_sl, &start_xy);
}

void LaneChangePath::GetBoundaryFromLaneChangeForbiddenZone(
    PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);

  if (is_clear_to_change_lane_) {
    is_exist_lane_change_start_position_ = false;
    return;
  }
  double lane_change_start_s = 0.0;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  if (is_exist_lane_change_start_position_) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_start_xy_, &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    lane_change_start_s =
        config_.lane_change_prepare_length() + init_sl_state_.first[0];

    // Update the lane_change_start_xy_ decided by lane_change_start_s
    GetLaneChangeStartPoint(reference_line, init_sl_state_.first[0],
                            lane_change_start_xy_);
  }

  // Remove the target lane out of the path-boundary, up to the decided S.
  if (lane_change_start_s < init_sl_state_.first[0]) {
    // If already passed the decided S, then return.
    return;
  }
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    if (curr_s > lane_change_start_s) {
      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    std::get<1>((*path_bound)[i]) =
        init_sl_state_.second[0] > curr_lane_left_width
            ? curr_lane_left_width + adc_half_width
            : std::get<1>((*path_bound)[i]);
    std::get<1>((*path_bound)[i]) = std::fmin(std::get<1>((*path_bound)[i]),
                                              init_sl_state_.second[0] - 0.1);
    std::get<2>((*path_bound)[i]) =
        init_sl_state_.second[0] < -curr_lane_right_width
            ? -curr_lane_right_width - adc_half_width
            : std::get<2>((*path_bound)[i]);
    std::get<2>((*path_bound)[i]) = std::fmax(std::get<2>((*path_bound)[i]),
                                              init_sl_state_.second[0] + 0.1);
  }
}

void LaneChangePath::UpdateStatus(double timestamp,
                                  ChangeLaneStatus::Status status_code,
                                  const std::string& path_id) {
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  AINFO << "UPDATEA FROM" << lane_change_status->DebugString() << "to";
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
  AINFO << lane_change_status->DebugString();
}

bool LaneChangePath::HysteresisFilter(const double obstacle_distance,
                                      const double safe_distance,
                                      const double distance_buffer,
                                      const bool is_obstacle_blocking) {
  if (is_obstacle_blocking) {
    return obstacle_distance < safe_distance + distance_buffer;
  } else {
    return obstacle_distance < safe_distance - distance_buffer;
  }
}

void LaneChangePath::SetPathInfo(PathData* const path_data) {
  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      *path_data, PathData::PathPointType::IN_LANE, &path_decision);
  PathAssessmentDeciderUtil::SetPathPointType(*reference_line_info_, *path_data,
                                              true, &path_decision);
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool LaneChangePath::CheckLastFrameSucceed(
    const apollo::planning::Frame* const last_frame) {
  if (last_frame) {
    for (const auto& reference_line_info : last_frame->reference_line_info()) {
      if (!reference_line_info.IsChangeLanePath()) {
        continue;
      }
      const auto history_trajectory_type =
          reference_line_info.trajectory_type();
      if (history_trajectory_type == ADCTrajectory::SPEED_FALLBACK) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
