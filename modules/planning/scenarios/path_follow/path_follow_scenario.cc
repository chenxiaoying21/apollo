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

#include "modules/planning/scenarios/path_follow/path_follow_scenario.h"

#include "modules/common_msgs/external_command_msgs/path_follow_command.pb.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool PathFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  // Transfer to PathFollowScenario if input planning command contains
  // PathFollowCommand.
  const auto& planning_command = frame.local_view().planning_command;
  if (!planning_command->has_custom_command()) {
    return false;
  }
  if (!planning_command->custom_command()
           .Is<external_command::PathFollowCommand>()) {
    return false;
  }
  external_command::PathFollowCommand unpacked_command;
  if (!planning_command->custom_command().UnpackTo(&unpacked_command)) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
