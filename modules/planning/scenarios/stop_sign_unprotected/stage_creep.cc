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

/**
 * @file
 **/
#include "modules/planning/scenarios/stop_sign_unprotected/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/common/creep_util/creep_decider.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool StopSignUnprotectedStageCreep::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  CHECK_NOTNULL(context);
  bool ret = Stage::Init(config, injector, config_dir, context);
  if (!ret) {
    AERROR << Name() << "init failed!";
    return false;
  }
  creep_decider_ = std::make_shared<CreepDecider>(
      GetContextAs<StopSignUnprotectedContext>()
          ->scenario_config.creep_decider_config(),
      injector);
  return ret;
}

Stage::StageStatus StopSignUnprotectedStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<StopSignUnprotectedContext>();
  const ScenarioStopSignUnprotectedConfig& scenario_config =
      context->scenario_config;

  if (!pipeline_config_.enabled()) {
    return FinishStage();
  }

  // Run creep decider.
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      break;
    }

    const auto ret = creep_decider_->Process(frame, &reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStageCreep planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = context->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  const double stop_sign_end_s = current_stop_sign_overlap->end_s;
  const double wait_time = Clock::NowInSeconds() - context->creep_start_time;
  const double timeout_sec = scenario_config.creep_timeout_sec();

  double creep_stop_s = stop_sign_end_s + creep_decider_->FindCreepDistance(
                                              *frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (creep_decider_->CheckCreepDone(*frame, reference_line_info,
                                     stop_sign_end_s, wait_time, timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus StopSignUnprotectedStageCreep::FinishStage() {
  next_stage_ = "STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE";
  return Stage::FINISHED;
}

}  // namespace planning
}  // namespace apollo
