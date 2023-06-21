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
 * @file speed_command_processor.cc
 **/

#include "modules/external_command/command_processor/speed_command_processor/speed_command_processor.h"

#include "modules/external_command/command_processor/command_processor_base/proto/command_processor_config.pb.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"

namespace apollo {
namespace external_command {

bool SpeedCommandProcessor::Init(const std::shared_ptr<cyber::Node>& node) {
  if (!CommandProcessorBase::Init(node)) {
    return false;
  }
  message_reader_ = MessageReader::Instance();
  const auto& config = GetProcessorConfig();
  command_service_ = node->CreateService<SpeedCommand, CommandStatus>(
      config.input_command_name(),
      [this](const std::shared_ptr<SpeedCommand>& command,
             std::shared_ptr<CommandStatus>& status) {
        this->OnCommand(command, status);
      });
  // Create writer for output command.
  CHECK(config.output_command_name().size() > 0 &&
        config.input_command_status_name().size() > 0);
  planning_command_writer_ =
      node->CreateWriter<apollo::planning::PlanningCommand>(
          config.output_command_name().Get(0));
  planning_command_status_name_ = config.input_command_status_name().Get(0);
  message_reader_->RegisterMessage<CommandStatus>(
      planning_command_status_name_);
  return true;
}

bool SpeedCommandProcessor::GetCommandStatus(int64_t command_id,
                                             CommandStatus* status) const {
  CHECK_NOTNULL(status);
  if (command_id != last_received_command_.command_id()) {
    return false;
  }
  status->set_command_id(command_id);
  // todo: Get planning speed setting status.
  status->set_status(apollo::external_command::CommandStatusType::FINISHED);
  return true;
}

void SpeedCommandProcessor::OnCommand(
    const std::shared_ptr<SpeedCommand>& command,
    std::shared_ptr<CommandStatus>& status) {
  auto planning_command = std::make_shared<apollo::planning::PlanningCommand>();
  auto custom_command = planning_command->mutable_custom_command();
  custom_command->PackFrom(*command);
  planning_command_writer_->Write(planning_command);
  last_received_command_.CopyFrom(*command);
  status->set_command_id(command->command_id());
  status->set_status(apollo::external_command::CommandStatusType::RUNNING);
}

}  // namespace external_command
}  // namespace apollo
