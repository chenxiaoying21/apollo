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

/**
 * @file
 **/

#include "modules/planning/tasks/common/path_generation.h"

namespace apollo {
namespace planning {

apollo::common::Status PathGeneration::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

apollo::common::Status PathGeneration::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}

void PathGeneration::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  ACHECK(!path_boundaries.empty());
  CHECK_NOTNULL(reference_line_info);

  // Take the left and right path boundaries, and transform them into two
  // PathData so that they can be displayed in simulator.
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(std::get<0>(path_bound_point));
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);

    frenet_frame_point.set_l(std::get<1>(path_bound_point));
    frenet_frame_right_boundaries.push_back(frenet_frame_point);
    frenet_frame_point.set_l(std::get<2>(path_bound_point));
    frenet_frame_left_boundaries.push_back(frenet_frame_point);
  }

  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  PathData left_path_data;
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  // Insert the transformed PathData into the simulator display.
  auto* ptr_display_path_1 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_1->set_name(std::string("planning_path_boundary_1_") +
                               debug_name);
  ptr_display_path_1->mutable_path_point()->CopyFrom(
      {left_path_data.discretized_path().begin(),
       left_path_data.discretized_path().end()});
  auto* ptr_display_path_2 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_2->set_name(std::string("planning_path_boundary_2_") +
                               debug_name);
  ptr_display_path_2->mutable_path_point()->CopyFrom(
      {right_path_data.discretized_path().begin(),
       right_path_data.discretized_path().end()});
}

void PathGeneration::RecordDebugInfo(
    const PathData& path_data, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_optimized_path->set_name(std::string("candidate_path_") + debug_name);
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}
void PathGeneration::GetStartPointSLState() {
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  common::TrajectoryPoint planning_start_point = frame_->PlanningStartPoint();
  if (FLAGS_use_front_axe_center_in_path_planning) {
    double front_to_rear_axe_distance =
        apollo::common::VehicleConfigHelper::GetConfig()
            .vehicle_param()
            .wheel_base();
    planning_start_point.mutable_path_point()->set_x(
        planning_start_point.path_point().x() +
        front_to_rear_axe_distance *
            std::cos(planning_start_point.path_point().theta()));
    planning_start_point.mutable_path_point()->set_y(
        planning_start_point.path_point().y() +
        front_to_rear_axe_distance *
            std::sin(planning_start_point.path_point().theta()));
  }
  ADEBUG << std::fixed << "Plan at the starting point: x = "
         << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y()
         << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  init_sl_state_ = reference_line.ToFrenetFrame(planning_start_point);
}
}  // namespace planning
}  // namespace apollo
