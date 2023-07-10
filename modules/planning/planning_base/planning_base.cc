/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base/planning_base.h"

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

PlanningBase::~PlanningBase() {}

Status PlanningBase::Init(const PlanningConfig& config) {
  injector_->planning_context()->Init();
  return Status::OK();
}

bool PlanningBase::IsPlanningFinished() {
  if (nullptr == frame_ || frame_->reference_line_info().empty() ||
      nullptr == local_view_.planning_command) {
    return true;
  }
  const auto& reference_line_info = frame_->reference_line_info().front();
  // Check if the ReferenceLineInfo is the last passage.
  const auto& reference_points =
      reference_line_info.reference_line().reference_points();
  if (reference_points.empty()) {
    return true;
  }
  const auto& last_reference_point = reference_points.back();
  const auto& lane_way_points = last_reference_point.lane_waypoints();
  if (lane_way_points.empty()) {
    return true;
  }
  // Get the end lane way point.
  if (nullptr == frame_->local_view().end_lane_way_point) {
    return true;
  }
  // If the lane id of RoutingRequest end point and the last way point of
  // ReferenceLineInfo is the same, ReferenceLineInfo is on the last passage.
  const std::string& end_lane_id_of_routing_request =
      frame_->local_view().end_lane_way_point->id();
  bool is_reference_line_on_last_passage = false;
  for (const auto& way_point : lane_way_points) {
    if (way_point.lane->id().id() == end_lane_id_of_routing_request) {
      is_reference_line_on_last_passage = true;
      break;
    }
  }
  if (!is_reference_line_on_last_passage) {
    return false;
  }
  // Check if vehicle reached end of the ReferenceLineInfo
  double adc_s = reference_line_info.AdcSlBoundary().end_s();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  const double current_vehicle_speed =
      injector_->vehicle_state()->linear_velocity();
  const double kStopDistanceTolerance = 2.0;
  if (current_vehicle_speed <= max_adc_stop_speed &&
      adc_s + kStopDistanceTolerance >
          reference_line_info.reference_line().Length()) {
    return true;
  }
  return false;
}

void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (local_view_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  trajectory_pb->mutable_routing_header()->CopyFrom(
      local_view_.planning_command->header());
}

}  // namespace planning
}  // namespace apollo
