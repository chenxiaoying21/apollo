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

#include <list>
#include <memory>
#include <string>

#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/tasks/lane_change_decider/proto/lane_change_decider.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class LaneChangeDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  common::Status Process(
      Frame* frame,
      ReferenceLineInfo* const current_reference_line_info) override;

  void UpdateStatus(ChangeLaneStatus::Status status_code,
                    const std::string& path_id);
  void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code,
                    const std::string& path_id);

  void PrioritizeChangeLane(
      const bool is_prioritize_change_lane,
      std::list<ReferenceLineInfo>* reference_line_info) const;

  void RemoveChangeLane(
      std::list<ReferenceLineInfo>* reference_line_info) const;

  std::string GetCurrentPathId(
      const std::list<ReferenceLineInfo>& reference_line_info) const;

 private:
  LaneChangeDeciderConfig config_;  // the config the task
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneChangeDecider, Task)

}  // namespace planning
}  // namespace apollo
