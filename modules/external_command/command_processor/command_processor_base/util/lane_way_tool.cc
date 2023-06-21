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
 * @file lane_way_tool.cc
 */

#include "modules/external_command/command_processor/command_processor_base/util/lane_way_tool.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/external_command/command_processor/command_processor_base/util/message_reader.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace external_command {

LaneWayTool::LaneWayTool(const std::shared_ptr<cyber::Node> &node)
    : hdmap_(hdmap::HDMapUtil::BaseMapPtr()),
      message_reader_(MessageReader::Instance()) {
  message_reader_->RegisterMessage<apollo::localization::LocalizationEstimate>(
      FLAGS_localization_topic);
}

bool LaneWayTool::ConvertToLaneWayPoint(
    const Pose &pose, apollo::routing::LaneWaypoint *lane_way_point) const {
  hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  common::PointENU point;
  point.set_x(pose.x());
  point.set_y(pose.y());
  if (pose.has_heading()) {
    static constexpr double kSearchRadius = 3.0;
    static constexpr double kMaxHeadingDiff = 1.0;
    // Get the lane nearest to the pose with heading check and update the lane
    // info in LaneWayPoint.
    if (nullptr == hdmap_ ||
        hdmap_->GetNearestLaneWithHeading(point, kSearchRadius, pose.heading(),
                                          kMaxHeadingDiff, &nearest_lane,
                                          &nearest_s, &nearest_l) < 0) {
      AERROR << "Failed to get nearest lane with heading of pose "
             << pose.DebugString();
      return false;
    }
  } else {
    // Get the lane nearest to the pose without heading check and update the
    // lane info in LaneWayPoint.
    if (nullptr == hdmap_ ||
        hdmap_->GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l) <
            0) {
      AERROR << "Failed to get nearest lane of " << pose.DebugString();
      return false;
    }
  }
  // Check the lane type.
  if (nearest_lane->lane().type() != hdmap::Lane::CITY_DRIVING) {
    AERROR << pose.DebugString() << " Lane type is not correct "
           << apollo::hdmap::Lane::LaneType_Name(nearest_lane->lane().type());
    return false;
  }
  // Update the LaneWayPoint info.
  lane_way_point->set_id(nearest_lane->id().id());
  lane_way_point->set_s(nearest_s);
  auto *lane_way_pose = lane_way_point->mutable_pose();
  lane_way_pose->set_x(pose.x());
  lane_way_pose->set_y(pose.y());
  return true;
}

bool LaneWayTool::GetVehicleLaneWayPoint(
    apollo::routing::LaneWaypoint *lane_way_point) const {
  CHECK_NOTNULL(lane_way_point);
  // Get the current localization pose
  auto *localization =
      message_reader_->GetMessage<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic);
  if (nullptr == localization) {
    AERROR << "Cannot get vehicle location!";
    return false;
  }
  external_command::Pose pose;
  pose.set_x(localization->pose().position().x());
  pose.set_y(localization->pose().position().y());
  pose.set_heading(localization->pose().heading());
  return ConvertToLaneWayPoint(pose, lane_way_point);
}

}  // namespace external_command
}  // namespace apollo
