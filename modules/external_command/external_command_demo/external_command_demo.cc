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

#include <poll.h>

#include "modules/external_command/external_command_demo/external_command_demo.h"

#include "cyber/record/record_reader.h"

using apollo::external_command::CommandStatus;

ExternalCommandDemo::ExternalCommandDemo()
    : command_id_(0), module_name_("demo") {}

bool ExternalCommandDemo::Init() {
  action_command_client_ =
      node_->CreateClient<apollo::external_command::ActionCommand,
                          CommandStatus>("/apollo/external_command/action");
  chassis_command_client_ =
      node_->CreateClient<apollo::external_command::ChassisCommand,
                          CommandStatus>("/apollo/external_command/chassis");
  free_space_command_client_ =
      node_->CreateClient<apollo::external_command::FreeSpaceCommand,
                          CommandStatus>("/apollo/external_command/free_space");
  lane_follow_command_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          CommandStatus>(
          "/apollo/external_command/lane_follow");
  path_follow_command_client_ =
      node_->CreateClient<apollo::external_command::PathFollowCommand,
                          CommandStatus>(
          "/apollo/external_command/path_follow");
  speed_command_client_ =
      node_
          ->CreateClient<apollo::external_command::SpeedCommand, CommandStatus>(
              "/apollo/external_command/speed");
  valet_parking_command_client_ =
      node_->CreateClient<apollo::external_command::ValetParkingCommand,
                          CommandStatus>(
          "/apollo/external_command/valet_parking");
  return true;
}

bool ExternalCommandDemo::Proc() {
  int8_t revent = 0;  // short
  struct pollfd fd = {STDIN_FILENO, POLLIN, revent};
  switch (poll(&fd, 1, 100)) {
    case -1:
      std::cout << "Failed to read keyboard" << std::endl;
      return false;
    case 0:
      return true;
    default:
      std::string input_command_string = "";
      std::cin >> input_command_string;
      if (input_command_string == "pull_over") {
        // Pull over.
        SendActionCommand(
            apollo::external_command::ActionCommandType::PULL_OVER);
      } else if (input_command_string == "stop") {
        // Stop planning.
        SendActionCommand(apollo::external_command::ActionCommandType::STOP);
      } else if (input_command_string == "start") {
        // Start planning.
        SendActionCommand(apollo::external_command::ActionCommandType::START);
      } else if (input_command_string == "manual") {
        // Switch manual mode.
        SendActionCommand(
            apollo::external_command::ActionCommandType::SWITCH_TO_MANUAL);
      } else if (input_command_string == "auto") {
        // Switch auto mode.
        SendActionCommand(
            apollo::external_command::ActionCommandType::SWITCH_TO_AUTO);
      } else if (input_command_string == "vin") {
        // Send vin validation.
        SendActionCommand(apollo::external_command::ActionCommandType::VIN_REQ);
      } else if (input_command_string == "chassis") {
        // Send left turn signal.
        auto command =
            std::make_shared<apollo::external_command::ChassisCommand>();
        FillCommandHeader(command);
        command->mutable_basic_signal()->set_turn_signal(
            apollo::common::VehicleSignal::TURN_LEFT);
        std::cout << "Sending chassis command: " << command->DebugString()
                  << std::endl;
        auto response = chassis_command_client_->SendRequest(command);
        if (nullptr == response) {
          std::cout
              << "Command sending failed, please check the service is on!\n"
              << std::endl;
        } else {
          std::cout << "******Finish sending command.******\n" << std::endl;
        }
      } else if (input_command_string == "set_speed_low") {
        double speed = 2.0;
        SendSpeedCommand(speed);
      } else if (input_command_string == "set_speed_high") {
        double speed = 30.0;
        SendSpeedCommand(speed);
      } else if (input_command_string == "increase_speed") {
        double speed_factor = 1.2;
        SendSpeedFactorCommand(speed_factor);
      } else if (input_command_string == "decrease_speed") {
        double speed_factor = 0.8;
        SendSpeedFactorCommand(speed_factor);
      } else if (input_command_string == "lane") {
        apollo::external_command::Pose way_point;
        way_point.set_x(0.0);
        way_point.set_y(0.0);
        way_point.set_heading(0.0);
        std::vector<apollo::external_command::Pose> way_points;
        way_points.emplace_back(way_point);
        apollo::external_command::Pose end_pose;
        end_pose.set_x(10.0);
        end_pose.set_y(0.0);
        end_pose.set_heading(0.0);
        SendLaneFollowCommand(way_points, end_pose, 3.0);
      } else if (input_command_string == "path") {
        // Read planning data from record file and use the planning path points
        // as the path of PathFollowCommand.
        apollo::planning::ADCTrajectory record_planning_data;
        ReadPlanningDataFromRecord("/apollo/data/bag/demo.record",
                                   &record_planning_data);
        // Get the path points from record planning data.
        std::shared_ptr<apollo::external_command::PathFollowCommand>
            path_follow_command =
                std::make_shared<apollo::external_command::PathFollowCommand>();
        Convert(record_planning_data, path_follow_command->mutable_way_point());
        // Set header and command id of PathFollowCommand.
        FillCommandHeader(path_follow_command);
        // Set path boundary of path.
        auto path_boundary = path_follow_command->mutable_boundary_with_width();
        path_boundary->set_left_path_width(4.5);
        path_boundary->set_right_path_width(4.0);
        // Set target speed.
        path_follow_command->set_target_speed(5.0);
        auto response =
            path_follow_command_client_->SendRequest(path_follow_command);
        if (nullptr == response) {
          std::cout
              << "Command sending failed, please check the service is on!\n"
              << std::endl;
        } else {
          std::cout << "******Finish sending command.******\n" << std::endl;
        }
      } else if (input_command_string == "valet_parking") {
        std::string parking_spot_id = "451089045";
        double target_speed = 6.0;
        SendValetParkingCommand(parking_spot_id, target_speed);
      } else {
        std::cout << "Invalid input!" << std::endl;
      }
  }
  return true;
}

void ExternalCommandDemo::SendActionCommand(
    apollo::external_command::ActionCommandType action_command_type) {
  auto command = std::make_shared<apollo::external_command::ActionCommand>();
  FillCommandHeader(command);
  command->set_command(action_command_type);
  std::cout << "Sending action command: " << command->DebugString()
            << std::endl;
  auto response = action_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandDemo::SendSpeedCommand(double speed) {
  auto command = std::make_shared<apollo::external_command::SpeedCommand>();
  FillCommandHeader(command);
  command->set_target_speed(speed);
  std::cout << "Sending speed command: " << command->DebugString() << std::endl;
  auto response = speed_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandDemo::SendSpeedFactorCommand(double speed_factor) {
  auto command = std::make_shared<apollo::external_command::SpeedCommand>();
  FillCommandHeader(command);
  command->set_target_speed_factor(speed_factor);
  std::cout << "Sending speed factor command: " << command->DebugString()
            << std::endl;
  auto response = speed_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandDemo::SendLaneFollowCommand(
    const std::vector<apollo::external_command::Pose>& way_points,
    const apollo::external_command::Pose& end, double target_speed) {
  auto command =
      std::make_shared<apollo::external_command::LaneFollowCommand>();
  FillCommandHeader(command);
  // Copy way_points
  for (const auto& point : way_points) {
    auto way_point = command->add_way_point();
    way_point->CopyFrom(point);
  }
  // Copy end point
  command->mutable_end_pose()->CopyFrom(end);
  if (target_speed > 0.0) {
    command->set_target_speed(target_speed);
  }
  std::cout << "Sending lane follow command: " << command->DebugString()
            << std::endl;
  auto response = lane_follow_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandDemo::SendValetParkingCommand(
    const std::string& parking_spot_id, double target_speed) {
  auto command =
      std::make_shared<apollo::external_command::ValetParkingCommand>();
  FillCommandHeader(command);
  command->set_parking_spot_id(parking_spot_id);
  if (target_speed > 0.0) {
    command->set_target_speed(target_speed);
  }
  std::cout << "Sending valet parking command: " << command->DebugString()
            << std::endl;
  auto response = valet_parking_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandDemo::ReadPlanningDataFromRecord(
    const std::string& record_file,
    apollo::planning::ADCTrajectory* planning_trajectory) {
  std::cout << "ReadPlanningDataFromRecord: " << record_file << std::endl;
  apollo::cyber::record::RecordReader reader(record_file);
  if (!reader.IsValid()) {
    std::cout << "Fail to open " << record_file << std::endl;
    return;
  }

  apollo::cyber::record::RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == "/apollo/planning") {
      if (planning_trajectory->ParseFromString(message.content)) {
        return;
      }
    }
  }
}

void ExternalCommandDemo::Convert(
    const apollo::planning::ADCTrajectory& planning_trajectory,
    google::protobuf::RepeatedPtrField<apollo::external_command::Point>*
        waypoints) {
  waypoints->Clear();
  const auto& trajectory_points = planning_trajectory.trajectory_point();
  for (const auto input_point : trajectory_points) {
    auto output_point = waypoints->Add();
    output_point->set_x(input_point.path_point().x());
    output_point->set_y(input_point.path_point().y());
  }
}
