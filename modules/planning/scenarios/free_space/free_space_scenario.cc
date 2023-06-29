/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <vector>
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/open_space/utils/open_space_roi_util.h"
#include "modules/planning/scenarios/free_space/free_space_scenario.h"
#include "modules/planning/scenarios/free_space/stage_free_space.h"

namespace apollo {
namespace planning {
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::external_command::FreeSpaceCommand;

bool FreeSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector,
                             const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioFreeSpaceConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool FreeSpaceScenario::IsTransferable(const Scenario* const other_scenario,
                                       const Frame& frame) {
  const auto& planning_command = frame.local_view().planning_command;
  if (!planning_command->has_custom_command()) {
    AINFO << "no cmd" << planning_command->DebugString();
    return false;
  }
  if (!planning_command->custom_command().Is<FreeSpaceCommand>()) {
    AINFO << "not frsp" << planning_command->DebugString();
    return false;
  }
  context_.free_space_command.Clear();
  if (!planning_command->custom_command().UnpackTo(
          &context_.free_space_command)) {
    AERROR << "Free Space Command unpack error"
           << planning_command->DebugString();
    return false;
  }
  double adc_speed = frame.vehicle_state().linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (std::fabs(adc_speed) > max_adc_stop_speed) {
    AINFO << "adc not stopped" << adc_speed;
    return false;
  }
  apollo::common::math::Polygon2d ego_box(injector_->ego_info()->ego_box());
  for (const auto& roi_polygon : context_.free_space_command.parking_roi()) {
    std::vector<Vec2d> polygon_points;
    for (const auto& point : roi_polygon.point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
    Polygon2d polygon_roi(polygon_points);
    if (OpenSpaceRoiUtil::IsPolygonClockwise(polygon_points)) {
      if (!polygon_roi.Contains(ego_box)) {
        AERROR << "ego box is out of roi";
        return false;
      }
    } else {
      if (polygon_roi.HasOverlap(ego_box)) {
        AERROR << "ego box has collision with obs";
        return false;
      }
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
