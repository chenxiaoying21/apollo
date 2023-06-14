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
 * @file temp_routing_converter.cc
 */

#include "modules/temp_routing_converter/temp_routing_converter.h"

#include "modules/temp_routing_converter/proto/temp_routing_converter.pb.h"

/**
 * @namespace apollo::temp_routing_converter
 * @brief apollo::temp_routing_converter
 */
namespace apollo {
namespace temp_routing_converter {

using apollo::external_command::LaneFollowCommand;
using apollo::external_command::ValetParkingCommand;
using apollo::planning::PlanningCommand;
using apollo::temp_routing_converter::RoutingRequest;
using apollo::temp_routing_converter::RoutingResponse;

TempRoutingConverter::TempRoutingConverter() : command_id_(0) {}

bool TempRoutingConverter::Init() {
  // Load the config.
  TempRoutingConverterConfig config;
  if (!GetProtoConfig(&config)) {
    AERROR << "Unable to load TempRoutingConverter conf file: "
           << ConfigFilePath();
    return false;
  }
  // Init the readers and writers.
  routing_request_reader_ = node_->CreateReader<RoutingRequest>(
      config.routing_request_topic(),
      [this](const std::shared_ptr<RoutingRequest>& routing_request) {
        ADEBUG << "Received routing request: run callback.";
        OnRoutingRequest(routing_request);
      });
  routing_response_reader_ = node_->CreateReader<RoutingResponse>(
      config.routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing_response) {
        ADEBUG << "Received routing response: run callback.";
        OnRoutingResponse(routing_response);
      });

  lane_follow_command_writer_ = node_->CreateWriter<LaneFollowCommand>(
      config.lane_follow_command_topic());
  valet_parking_command_writer_ = node_->CreateWriter<ValetParkingCommand>(
      config.valet_parking_command_topic());
  planning_command_writer_ =
      node_->CreateWriter<PlanningCommand>(config.planning_command_topic());
  return true;
}

void TempRoutingConverter::OnRoutingRequest(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  if (routing_request->has_parking_info()) {
    auto valet_parking_command = std::make_shared<ValetParkingCommand>();
    CopyRoutingRequest<ValetParkingCommand>(routing_request, true,
                                            valet_parking_command.get());
    valet_parking_command->set_parking_spot_id(
        routing_request->parking_info().parking_space_id());
    valet_parking_command_writer_->Write(valet_parking_command);
  } else {
    auto lane_follow_command = std::make_shared<LaneFollowCommand>();
    CopyRoutingRequest<LaneFollowCommand>(routing_request, false,
                                          lane_follow_command.get());
    CHECK(routing_request->waypoint().size() > 0);
    lane_follow_command->mutable_end_pose()->CopyFrom(
        routing_request->waypoint().Get(routing_request->waypoint().size() -
                                        1));
    lane_follow_command_writer_->Write(lane_follow_command);
  }
}

void TempRoutingConverter::OnRoutingResponse(
    const std::shared_ptr<RoutingResponse>& routing_response) {
  auto planning_command = std::make_shared<PlanningCommand>();
  planning_command->set_command_id(++command_id_);
  auto lane_follow_command = planning_command->mutable_lane_follow_command();
  // Copy header.
  if (routing_response->has_header()) {
    planning_command->mutable_header()->CopyFrom(routing_response->header());
    lane_follow_command->mutable_header()->CopyFrom(routing_response->header());
  }
  // Copy road.
  lane_follow_command->mutable_road()->CopyFrom(routing_response->road());
  // Copy measurement.
  if (routing_response->has_measurement()) {
    lane_follow_command->mutable_measurement()->CopyFrom(
        routing_response->measurement());
  }
  // Copy RoutingRequest.
  auto input_routing_request = routing_response->routing_request();
  if (routing_response->has_routing_request()) {
    auto output_routing_request =
        lane_follow_command->mutable_routing_request();
    if (input_routing_request.has_header()) {
      output_routing_request->mutable_header()->CopyFrom(
          input_routing_request.header());
    }
    output_routing_request->mutable_waypoint()->CopyFrom(
        input_routing_request.waypoint());
    output_routing_request->mutable_blacklisted_road()->CopyFrom(
        input_routing_request.blacklisted_road());
    output_routing_request->set_broadcast(input_routing_request.broadcast());
  }
  // Copy map version
  lane_follow_command->set_map_version(routing_response->map_version());
  lane_follow_command->mutable_status()->CopyFrom(routing_response->status());
  // Process parkingInfo.
  if (input_routing_request.has_parking_info()) {
    auto parking_command = planning_command->mutable_parking_command();
    parking_command->mutable_header()->CopyFrom(routing_response->header());
    parking_command->set_parking_spot_id(
        input_routing_request.parking_info().parking_space_id());
  }
  planning_command_writer_->Write(planning_command);
}

}  // namespace temp_routing_converter
}  // namespace apollo