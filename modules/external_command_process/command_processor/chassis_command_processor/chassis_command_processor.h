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
 * @file chassis_command_processor.h
 **/

#pragma once

#include <memory>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/external_command_process/command_processor/command_processor_base/command_processor_base.h"

namespace apollo {
namespace external_command {

class ChassisCommandProcessor : public CommandProcessorBase {
 public:
  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  /**
   * @brief Get the command status.
   * @param command_id Id of the command.
   * @param status Output command status.
   */
  bool GetCommandStatus(uint64_t command_id,
                        CommandStatus* status) const override;

 private:
  /**
   * @brief Process the incoming command.
   * @param command Incoming command.
   * @param status The command process result before sending to planning module.
   */
  void OnCommand(const std::shared_ptr<ChassisCommand>& command,
                 std::shared_ptr<CommandStatus>& status);
  /**
   * @brief Callback for command status.
   * @param status The latest command status.
   */
  void OnChassisStatus(const std::shared_ptr<apollo::canbus::Chassis>& status);

  std::shared_ptr<cyber::Service<ChassisCommand, CommandStatus>>
      command_service_;
  std::shared_ptr<cyber::Writer<ChassisCommand>> chassis_command_writer_;
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>>
      chassis_status_reader_;

  ChassisCommand last_received_command_;
  apollo::canbus::Chassis latest_chassis_status_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::external_command::ChassisCommandProcessor, CommandProcessorBase)

}  // namespace external_command
}  // namespace apollo
