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

#include <string>
#include <vector>
#include "modules/planning/planning_base/open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/free_space/stage_free_space.h"

namespace apollo {
namespace planning {

bool StageFreeSpace::Init(const StagePipeline& config,
                          const std::shared_ptr<DependencyInjector>& injector,
                          const std::string& config_dir, void* context) {
  CHECK_NOTNULL(context);
  bool ret = Stage::Init(config, injector, config_dir, context);
  if (!ret) {
    AERROR << Name() << "init failed!";
    return false;
  }
  scenario_config_.CopyFrom(GetContextAs<FreeSpaceContext>()->scenario_config);
  return ret;
}

StageResult StageFreeSpace::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  auto* mutable_open_space_info = frame->mutable_open_space_info();
  const auto& free_space_command =
      GetContextAs<FreeSpaceContext>()->free_space_command;
  // Set free space boundary
  std::vector<std::vector<common::math::Vec2d>> roi_boundary;
  for (int i = 0; i < free_space_command.parking_roi_size(); i++) {
    roi_boundary.emplace_back();
    const auto& parking_roi = free_space_command.parking_roi(i);
    for (int j = 0; j < parking_roi.point_size(); j++) {
      roi_boundary.back().emplace_back(parking_roi.point(j).x(),
                                       parking_roi.point(j).y());
    }
  }
  // Set XY boundary
  OpenSpaceRoiUtil::GetRoiXYBoundary(
      roi_boundary, mutable_open_space_info->mutable_ROI_xy_boundary());
  // Set end pose
  auto* open_space_end_pose =
      mutable_open_space_info->mutable_open_space_end_pose();
  open_space_end_pose->push_back(free_space_command.parking_spot_pose().x());
  open_space_end_pose->push_back(free_space_command.parking_spot_pose().y());
  open_space_end_pose->push_back(
      free_space_command.parking_spot_pose().heading());
  open_space_end_pose->push_back(0);
  // Add obstacle boundary to roi_bound
  if (scenario_config_.enabled_perception_obstacles()) {
    OpenSpaceRoiUtil::LoadObstacles(
        scenario_config_.filtering_distance(),
        scenario_config_.perception_obstacle_buffer(),
        mutable_open_space_info->ROI_xy_boundary(), frame, &roi_boundary);
  }
  *mutable_open_space_info->mutable_obstacles_vertices_vec() = roi_boundary;
  // transform by origin point
  const auto& xy_boundary = mutable_open_space_info->ROI_xy_boundary();
  Vec2d origin_point(xy_boundary[0], xy_boundary[2]);
  OpenSpaceRoiUtil::TransformByOriginPoint(origin_point, 0.0,
                                           mutable_open_space_info);
  // Formulate as Ax < b
  if (!OpenSpaceRoiUtil::FormulateBoundaryConstraints(
          mutable_open_space_info)) {
    return StageResult(StageStatusType::ERROR);
  }
  mutable_open_space_info->set_is_on_open_space_trajectory(true);

  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageFreeSpace planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageFreeSpace::FinishStage() {
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
