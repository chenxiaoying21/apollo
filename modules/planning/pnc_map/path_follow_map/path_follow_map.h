/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file path_follow_map.h
 **/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/external_command_msgs/path_follow_command.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/pnc_map/pnc_map_base.h"

namespace apollo {
namespace planning {

class PathFollowMap : public PncMapBase {
 public:
  PathFollowMap();

  virtual ~PathFollowMap() = default;

  bool CanProcess(const planning::PlanningCommand &command) const override;

  bool UpdatePlanningCommand(const planning::PlanningCommand &command) override;
  /**
   * @brief use heuristic forward length and backward length
   */
  bool GetRouteSegments(
      const common::VehicleState &vehicle_state,
      std::list<apollo::hdmap::RouteSegments> *const route_segments) override;

  bool ExtendSegments(
      const apollo::hdmap::RouteSegments &segments, double start_s,
      double end_s,
      apollo::hdmap::RouteSegments *const truncated_segments) const override;

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const override;

 private:
  /**
   * @brief Record info of center curve to save computation time.
   */
  struct CurveInfo {
    CurveInfo(double s_value, double x_value, double y_value, double ds_value,
              double dx_value, double dy_value)
        : s(s_value),
          x(x_value),
          y(y_value),
          ds(ds_value),
          dx(dx_value),
          dy(dy_value) {}
    double s;
    double x;
    double y;
    double ds;
    double dx;
    double dy;
  };

  /**
   * @brief Check if the command can be processed by this map.
   * @param command The command to be checked.
   * @return True if the command can be processed.
   */
  bool IsValid(const planning::PlanningCommand &command) const override;
  /**
   * @brief Retrieve path follow command from PlanningCommand.
   * @param input_command The command to be retrieved.
   * @param output_command The PathFollowCommand command.
   * @return True if retriving succeed.
   */
  static bool GetPathFollowCommand(
      const planning::PlanningCommand &input_command,
      external_command::PathFollowCommand *output_command);
  /**
   * @brief Retrieve curve from point list.
   * @param points The input point list.
   * @param is_center_curve If "output_curve" is center curve.
   * @param output_curve The output curve.
   */
  void GetCurve(const ::google::protobuf::RepeatedPtrField<
                    apollo::external_command::Point> &points,
                bool is_center_curve, apollo::hdmap::Curve *output_curve);
  /**
   * @brief Get the boundary from the input boundary point list.
   * @param boundary_points The input boundary list.
   * @param output_boundary The output boundary.
   */
  void GetBoundary(const ::google::protobuf::RepeatedPtrField<
                       apollo::external_command::Point> &boundary_points,
                   apollo::hdmap::LaneBoundary *output_boundary);
  /**
   * @brief Get the boundary according to the center curve, left and right path
   * width.
   * @param center_curve Center curve.
   * @param left_path_width The width from the left boundary to center curve.
   * @param right_path_width The width from the right boundary to center curve.
   * @param left_boundary output left boundary.
   * @param right_boundary output right boundary.
   */
  void GetBoundary(const apollo::hdmap::Curve &center_curve,
                   double left_path_width, double right_path_width,
                   apollo::hdmap::LaneBoundary *left_boundary,
                   apollo::hdmap::LaneBoundary *right_boundary) const;
  /**
   * @brief Find the point on the lane nearest to the vehicle position.
   * @param adc_x x coordindate of vehicle.
   * @param adc_y y coordindate of vehicle.
   * @return The index of point on the lane nearest to the vehicle position.
   */
  size_t FindMapingPointOnLane(double adc_x, double adc_y);

  external_command::PathFollowCommand last_path_follow_command_;
  apollo::hdmap::LaneInfoConstPtr lane_info_;
  const std::string virtual_lane_name_;
  // Record s coordinate of center curve to save computation time.
  std::vector<CurveInfo> extra_center_info_;
  // Last index of center curve mapping to the vehicle position.
  size_t last_curve_index_of_vehicle_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathFollowMap,
                                     PncMapBase)

}  // namespace planning
}  // namespace apollo
