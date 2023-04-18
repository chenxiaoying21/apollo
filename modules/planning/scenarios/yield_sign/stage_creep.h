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

#pragma once

#include <memory>
#include <string>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/base_stage_creep.h"
#include "modules/planning/scenarios/yield_sign/yield_sign_scenario.h"

namespace apollo {
namespace planning {

struct YieldSignContext;
class CreepDecider;

class YieldSignStageCreep : public BaseStageCreep {
 public:
  bool Init(const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir, void* context) override;

  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

 private:
  /**
   * @brief Get the config of creep stage from ScenarioContext, to be overwrited
   * by the sub classes.
   *
   * @return config of creep stage
   */
  const CreepStageConfig& GetCreepStageConfig() const override;

  /**
   * @brief Get the overlap id of stage and the stop line distance according to
   * the frame and reference line information.
   *
   * @param frame current frame information
   * @param reference_line_info current reference line information
   * @param stop_line_s stop distance of the creep stage
   * @param overlap_id overlap id of current stage
   */
  void GetOverlapStopInfo(Frame* frame, ReferenceLineInfo* reference_line_info,
                          double* stop_line_s,
                          std::string* overlap_id) const override;

  Stage::StageStatus FinishStage();

  ScenarioYieldSignConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::YieldSignStageCreep,
                                     Stage)

}  // namespace planning
}  // namespace apollo
