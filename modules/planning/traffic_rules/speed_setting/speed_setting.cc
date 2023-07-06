/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file speed_setting.cc
 **/

#include "modules/planning/traffic_rules/speed_setting/speed_setting.h"

#include "modules/common_msgs/external_command_msgs/speed_command.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

Status SpeedSetting::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  if (!frame->local_view().planning_command->has_custom_command()) {
    return Status::OK();
  }
  const auto& custom_command =
      frame->local_view().planning_command->custom_command();
  if (!custom_command.Is<external_command::SpeedCommand>()) {
    return Status::OK();
  }
  external_command::SpeedCommand speed_command;
  if (!custom_command.UnpackTo(&speed_command)) {
    AERROR << "Unpack speed command failed!";
    return Status(common::PLANNING_ERROR);
  }
  if (speed_command.has_target_speed()) {
    reference_line_info->SetCruiseSpeed(speed_command.target_speed());
  } else if (speed_command.has_target_speed_factor()) {
    // Calculate the target speed with speed_factor.
    double target_speed = reference_line_info->GetBaseCruiseSpeed() *
                          speed_command.target_speed_factor();
    reference_line_info->SetCruiseSpeed(target_speed);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
