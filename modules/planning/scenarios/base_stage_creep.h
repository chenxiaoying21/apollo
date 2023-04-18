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
 * @file base_stage_creep.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace common {
class Status;
}  // namespace common
}  // namespace apollo

namespace apollo {
namespace planning {

class Frame;
class ReferenceLineInfo;
class CreepStageConfig;

class BaseStageCreep : public Stage {
 public:
  common::Status ProcessCreep(Frame* frame,
                              ReferenceLineInfo* reference_line_info) const;

  bool CheckCreepDone(const Frame& frame,
                      const ReferenceLineInfo& reference_line_info,
                      const double stop_sign_overlap_end_s,
                      const double wait_time_sec, const double timeout_sec);

  double FindCreepDistance(const Frame& frame,
                           const ReferenceLineInfo& reference_line_info) const;

 private:
  /**
   * @brief Get the config of creep stage from ScenarioContext, to be overwrited
   * by the sub classes.
   *
   * @return config of creep stage
   */
  virtual const CreepStageConfig& GetCreepStageConfig() const = 0;

  /**
   * @brief Get the overlap id of stage and the stop line distance according to
   * the frame and reference line information.
   *
   * @param frame current frame information
   * @param reference_line_info current reference line information
   * @param stop_line_s stop distance of the creep stage
   * @param overlap_id overlap id of current stage
   */
  virtual void GetOverlapStopInfo(Frame* frame,
                                  ReferenceLineInfo* reference_line_info,
                                  double* stop_line_s,
                                  std::string* overlap_id) const = 0;

  static constexpr const char* CREEP_VO_ID_PREFIX_ = "CREEP_";
};

}  // namespace planning
}  // namespace apollo
