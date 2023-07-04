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

#pragma once

#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

namespace apollo {
namespace planning {

class PathFollowScenario : public LaneFollowScenario {
 public:
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathFollowScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
