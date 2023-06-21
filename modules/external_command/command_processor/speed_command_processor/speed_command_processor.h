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
 * @file speed_command_processor.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/speed_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/external_command/command_processor/command_processor_base/command_processor_base.h"

namespace apollo {
namespace external_command {

class MessageReader;

class SpeedCommandProcessor : public CommandProcessorBase {
 public:
  bool Init(const std::shared_ptr<cyber::Node>& node) override;
  /**
   * @brief Get the command status.
   * @param command_id Id of the command.
   * @param status Output command status.
   */
  bool GetCommandStatus(int64_t command_id,
                        CommandStatus* status) const override;

 private:
  /**
   * @brief Process the incoming lane follow command. Search the routing to end
   * point and send to planning module.
   * @param command Incoming lane follow command.
   * @param status The command process result before sending to planning module.
   */
  void OnCommand(const std::shared_ptr<SpeedCommand>& command,
                 std::shared_ptr<CommandStatus>& status);

  std::shared_ptr<cyber::Service<SpeedCommand, CommandStatus>> command_service_;
  std::shared_ptr<cyber::Writer<apollo::planning::PlanningCommand>>
      planning_command_writer_;

  MessageReader* message_reader_;

  SpeedCommand last_received_command_;
  std::string planning_command_status_name_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::external_command::SpeedCommandProcessor, CommandProcessorBase)

}  // namespace external_command
}  // namespace apollo
