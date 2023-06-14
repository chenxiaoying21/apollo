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
 * @file temp_routing_converter_process_component.h
 */

#pragma once

#include <vector>

#include "cyber/component/component.h"

#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/valet_parking_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/temp_routing_converter/proto/routing.pb.h"

/**
 * @namespace apollo::temp_routing_converter
 * @brief apollo::temp_routing_converter
 */
namespace apollo {
namespace temp_routing_converter {

/**
 * @class TempRoutingConverter
 *
 * @brief The external interface for processing external commands.
 */
class TempRoutingConverter : public cyber::Component<> {
 public:
  TempRoutingConverter();

  ~TempRoutingConverter() = default;

  bool Init() override;

 private:
  void OnRoutingRequest(
      const std::shared_ptr<apollo::temp_routing_converter::RoutingRequest>&
          routing_request);

  void OnRoutingResponse(
      const std::shared_ptr<apollo::temp_routing_converter::RoutingResponse>&
          routing_response);

  template <typename T>
  void CopyRoutingRequest(
      const std::shared_ptr<RoutingRequest>& routing_request,
      bool is_copy_end_point, T* message);

  std::shared_ptr<cyber::Reader<apollo::temp_routing_converter::RoutingRequest>>
      routing_request_reader_;
  std::shared_ptr<
      cyber::Reader<apollo::temp_routing_converter::RoutingResponse>>
      routing_response_reader_;

  // Coverted from RoutingRequest without parking id.
  std::shared_ptr<cyber::Writer<apollo::external_command::LaneFollowCommand>>
      lane_follow_command_writer_;
  // Coverted from RoutingRequest with parking id.
  std::shared_ptr<cyber::Writer<apollo::external_command::ValetParkingCommand>>
      valet_parking_command_writer_;
  // Coverted from RoutingResponse.
  std::shared_ptr<cyber::Writer<apollo::planning::PlanningCommand>>
      planning_command_writer_;
  uint64_t command_id_;
};

template <typename T>
void TempRoutingConverter::CopyRoutingRequest(
    const std::shared_ptr<RoutingRequest>& routing_request,
    bool is_copy_end_point, T* message) {
  if (routing_request->has_header()) {
    message->mutable_header()->CopyFrom(routing_request->header());
  }
  message->set_command_id(++command_id_);
  const size_t way_point_num = routing_request->waypoint().size();
  size_t current_index = 0;
  for (auto way_point : routing_request->waypoint()) {
    message->add_way_point()->CopyFrom(way_point);
    if (!is_copy_end_point && ++current_index >= way_point_num) {
      break;
    }
  }
}

CYBER_REGISTER_COMPONENT(TempRoutingConverter)

}  // namespace temp_routing_converter
}  // namespace apollo