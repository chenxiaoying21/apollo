/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "modules/common_msgs/external_command_msgs/geometry.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "cyber/cyber.h"

namespace apollo {
namespace hdmap {
class HDMap;
}  // namespace hdmap
namespace routing {
class LaneWaypoint;
}  // namespace routing
namespace localization {
class LocalizationEstimate;
}  // namespace localization
}  // namespace apollo

namespace apollo {            // namespace apollo
namespace external_command {  // namespace external_command

// class Pose;

class LaneWayTool {
 public:
  LaneWayTool(const std::shared_ptr<cyber::Node> &node);

  /**
   * @brief Convert the pose with (x,y) and heading(or not) to lane_way_point.
   * @param pose The pose to be converted.
   * @param lane_way_point convert result of "lane_way_point".
   * @return True if no error occurs.
   */
  bool ConvertToLaneWayPoint(
      const apollo::external_command::Pose &pose,
      apollo::routing::LaneWaypoint *lane_way_point) const;
  /**
   * @brief Get the current vehicle pose and find the mapping point to the
   * nearest lane.
   * @param lane_way_point The raw command data.
   * @return True if no error occurs.
   */
  bool GetVehicleLaneWayPoint(
      apollo::routing::LaneWaypoint *lane_way_point) const;

 private:
  const hdmap::HDMap *hdmap_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
};

}  // namespace external_command
}  // namespace apollo
