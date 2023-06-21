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

#include <limits>

#include "modules/external_command/temp_routing_converter/temp_routing_converter.h"

#include "modules/external_command/temp_routing_converter/proto/temp_routing_converter.pb.h"
#include "modules/map/hdmap/hdmap_util.h"

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

TempRoutingConverter::TempRoutingConverter()
    : command_id_(0), hdmap_(nullptr) {}

bool TempRoutingConverter::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  // Load the config.
  TempRoutingConverterConfig config;
  if (!GetProtoConfig(&config)) {
    AERROR << "Unable to load TempRoutingConverter conf file: "
           << ConfigFilePath();
    return false;
  }
  // Init the readers, writers and service clients.
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
  lane_follow_command_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          apollo::external_command::CommandStatus>(
          config.lane_follow_command_topic());
  valet_parking_command_client_ =
      node_->CreateClient<apollo::external_command::ValetParkingCommand,
                          apollo::external_command::CommandStatus>(
          config.valet_parking_command_topic());
  planning_command_writer_ =
      node_->CreateWriter<PlanningCommand>(config.planning_command_topic());
  return true;
}

void TempRoutingConverter::OnRoutingRequest(
    const std::shared_ptr<RoutingRequest>& routing_request) {
  // Delay sending routing request and wait for localization updated.
  cyber::SleepFor(std::chrono::microseconds(500));
  if (routing_request->has_parking_info()) {
    auto valet_parking_command = std::make_shared<ValetParkingCommand>();
    // Copy the way points from RoutingRequest.
    int way_point_size = routing_request->waypoint().size();
    if (way_point_size > 0) {
      CopyRoutingRequest<ValetParkingCommand>(
          routing_request, 0, routing_request->waypoint().size(),
          valet_parking_command.get());
    }
    valet_parking_command->set_parking_spot_id(
        routing_request->parking_info().parking_space_id());
    auto response =
        valet_parking_command_client_->SendRequest(valet_parking_command);
    if (nullptr == response) {
      AERROR << "Failed to send request of valet parking!";
    }
  } else {
    auto lane_follow_command = std::make_shared<LaneFollowCommand>();
    // Copy the way points from RoutingRequest.
    int way_point_size = routing_request->waypoint().size();
    CHECK_GT(way_point_size, 0);
    if (way_point_size > 1) {
      CopyRoutingRequest<LaneFollowCommand>(
          routing_request, 0, way_point_size - 1, lane_follow_command.get());
    }
    // Copy the end point.
    Convert(routing_request->waypoint().Get(way_point_size - 1),
            lane_follow_command->mutable_end_pose());
    auto response =
        lane_follow_command_client_->SendRequest(lane_follow_command);
    if (nullptr == response) {
      AERROR << "Failed to get response of lane follow!";
    }
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

void TempRoutingConverter::Convert(
    const apollo::routing::LaneWaypoint& lane_way_point,
    apollo::external_command::Pose* pose) const {
  CHECK(lane_way_point.has_pose());
  const auto& way_pose = lane_way_point.pose();
  pose->set_x(way_pose.x());
  pose->set_y(way_pose.y());
  if (lane_way_point.has_heading()) {
    pose->set_heading(lane_way_point.heading());
    return;
  }
  // Get heading according to lane id
  if (lane_way_point.has_id()) {
    pose->set_heading(
        GetNearestHeading(lane_way_point.id(), way_pose.x(), way_pose.y()));
  }
}

double TempRoutingConverter::GetNearestHeading(const std::string& lane_id,
                                               double x, double y) const {
  hdmap::Id id;
  id.set_id(lane_id);
  auto lane = hdmap_->GetLaneById(id);
  AINFO << "hdmap lane id " << lane_id << " " << hdmap::BaseMapFile();
  CHECK_NOTNULL(lane);
  common::math::Vec2d proj_pt(0.0, 0.0);
  common::math::Vec2d point(x, y);
  double s_offset = 0.0;
  int s_offset_index = 0;
  lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
  return lane->headings()[s_offset_index];
}

}  // namespace temp_routing_converter
}  // namespace apollo
