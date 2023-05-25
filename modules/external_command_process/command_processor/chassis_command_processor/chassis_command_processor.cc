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
 * @file chassis_command_processor.cc
 **/

#include "modules/external_command_process/command_processor/chassis_command_processor/chassis_command_processor.h"

#include "modules/external_command_process/command_processor/command_processor_base/proto/command_processor_config.pb.h"

namespace apollo {
namespace external_command {

bool ChassisCommandProcessor::Init(const std::shared_ptr<cyber::Node>& node) {
  if (!CommandProcessorBase::Init(node)) {
    return false;
  }
  const auto& config = GetProcessorConfig();
  command_service_ = node->CreateService<ChassisCommand, CommandStatus>(
      config.input_command_name(),
      [this](const std::shared_ptr<ChassisCommand>& command,
             std::shared_ptr<CommandStatus>& status) {
        this->OnCommand(command, status);
      });
  // Create writer for output command.
  CHECK(config.output_command_name().size() > 0);
  chassis_command_writer_ =
      node->CreateWriter<ChassisCommand>(config.output_command_name().Get(0));
  chassis_status_reader_ = node->CreateReader<apollo::canbus::Chassis>(
      config.input_command_status_name().Get(0),
      [this](const std::shared_ptr<apollo::canbus::Chassis>& status) {
        this->OnChassisStatus(status);
      });
  return true;
}

bool ChassisCommandProcessor::GetCommandStatus(uint64_t command_id,
                                               CommandStatus* status) const {
  CHECK_NOTNULL(status);
  if (command_id != last_received_command_.command_id()) {
    return false;
  }
  // Process basic vehicle signals.
  if (last_received_command_.has_basic_signal()) {
    const auto& basic_signal = last_received_command_.basic_signal();
    // Check turn signal.
    if (basic_signal.has_turn_signal()) {
      if (!latest_chassis_status_.has_signal() ||
          !latest_chassis_status_.signal().has_turn_signal() ||
          latest_chassis_status_.signal().turn_signal() !=
              basic_signal.turn_signal()) {
        status->set_status(
            apollo::external_command::CommandStatusType::RUNNING);
        return true;
      }
    }
    // Check high beam.
    if (basic_signal.has_high_beam()) {
      if (!latest_chassis_status_.has_signal() ||
          !latest_chassis_status_.signal().has_high_beam() ||
          latest_chassis_status_.signal().high_beam() !=
              basic_signal.high_beam()) {
        status->set_status(
            apollo::external_command::CommandStatusType::RUNNING);
        return true;
      }
    }
    // Check low beam.
    if (basic_signal.has_low_beam()) {
      if (!latest_chassis_status_.has_signal() ||
          !latest_chassis_status_.signal().has_low_beam() ||
          latest_chassis_status_.signal().low_beam() !=
              basic_signal.low_beam()) {
        status->set_status(
            apollo::external_command::CommandStatusType::RUNNING);
        return true;
      }
    }
    // Check horn.
    if (basic_signal.has_horn()) {
      if (!latest_chassis_status_.has_signal() ||
          !latest_chassis_status_.signal().has_horn() ||
          latest_chassis_status_.signal().horn() != basic_signal.horn()) {
        status->set_status(
            apollo::external_command::CommandStatusType::RUNNING);
        return true;
      }
    }
    // Check emergency light.
    if (basic_signal.has_emergency_light()) {
      if (!latest_chassis_status_.has_signal() ||
          !latest_chassis_status_.signal().has_emergency_light() ||
          latest_chassis_status_.signal().emergency_light() !=
              basic_signal.emergency_light()) {
        status->set_status(
            apollo::external_command::CommandStatusType::RUNNING);
        return true;
      }
    }
  }
  status->set_status(apollo::external_command::CommandStatusType::FINISHED);
  return true;
}

void ChassisCommandProcessor::OnCommand(
    const std::shared_ptr<ChassisCommand>& command,
    std::shared_ptr<CommandStatus>& status) {
  chassis_command_writer_->Write(command);
  status->set_status(apollo::external_command::CommandStatusType::RUNNING);
  last_received_command_.CopyFrom(*command);
}

void ChassisCommandProcessor::OnChassisStatus(
    const std::shared_ptr<apollo::canbus::Chassis>& status) {
  CHECK_NOTNULL(status);
  CHECK_NOTNULL(status);
  latest_chassis_status_.CopyFrom(*status);
}

}  // namespace external_command
}  // namespace apollo
