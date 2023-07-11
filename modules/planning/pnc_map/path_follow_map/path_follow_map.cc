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
 * @file path_follow_map.cc
 **/

#include "modules/planning/pnc_map/path_follow_map/path_follow_map.h"

#include "modules/common_msgs/map_msgs/map_lane.pb.h"
#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

PathFollowMap::PathFollowMap()
    : end_lane_way_point_(nullptr),
      lane_info_(nullptr),
      lane_(nullptr),
      virtual_lane_name_("virtual_lane_path_follow"),
      last_curve_index_of_vehicle_(0) {}

bool PathFollowMap::CanProcess(const planning::PlanningCommand &command) const {
  external_command::PathFollowCommand path_follow_command;
  return GetPathFollowCommand(command, &path_follow_command);
}

bool PathFollowMap::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  last_curve_index_of_vehicle_ = 0;
  extra_center_info_.clear();
  lane_info_ = nullptr;
  lane_ = nullptr;
  if (!GetPathFollowCommand(command, &last_path_follow_command_)) {
    AERROR << "Unpack command failed!";
    return false;
  }
  lane_ = std::make_shared<apollo::hdmap::Lane>();
  // Generate RouteSegments(with with virtual lane) from PlanningCommand.
  lane_->mutable_id()->set_id(virtual_lane_name_);
  // Get center curve.
  GetCurve(last_path_follow_command_.way_point(), true,
           lane_->mutable_central_curve());
  // PathBoundary is given, get left and right boundary.
  if (last_path_follow_command_.has_path_boundary()) {
    GetBoundary(last_path_follow_command_.path_boundary().left_boundary(),
                lane_->mutable_left_boundary());
    GetBoundary(last_path_follow_command_.path_boundary().right_boundary(),
                lane_->mutable_right_boundary());
  } else if (last_path_follow_command_.has_boundary_with_width()) {
    // Left and right path width is given, get left and right boundary.
    GetBoundary(
        lane_->central_curve(),
        last_path_follow_command_.boundary_with_width().left_path_width(),
        last_path_follow_command_.boundary_with_width().right_path_width(),
        lane_->mutable_left_boundary(), lane_->mutable_right_boundary());
  } else {
    // Path boundary is not given, generate the default path boundary which is
    // barely enough to pass by and the vehicle should stop when there is an
    // obstacle on the path.
    const auto &vehicle_param = apollo::common::VehicleConfigHelper::Instance()
                                    ->GetConfig()
                                    .vehicle_param();
    const double width_buffer = 0.2;
    double default_path_width = vehicle_param.width() / 2.0 + width_buffer;
    GetBoundary(lane_->central_curve(), default_path_width, default_path_width,
                lane_->mutable_left_boundary(),
                lane_->mutable_right_boundary());
  }
  lane_->set_length(lane_->central_curve().segment().Get(0).length());
  lane_->set_type(apollo::hdmap::Lane::CITY_DRIVING);
  lane_->set_turn(apollo::hdmap::Lane::NO_TURN);
  lane_->set_direction(apollo::hdmap::Lane::FORWARD);
  lane_info_ = std::make_shared<apollo::hdmap::LaneInfo>(*lane_);
  // Set "end_lane_way_point".
  end_lane_way_point_ = std::make_shared<apollo::routing::LaneWaypoint>();
  end_lane_way_point_->set_id(virtual_lane_name_);
  end_lane_way_point_->set_s(lane_->length());
  if (!extra_center_info_.empty()) {
    const auto &last_point = extra_center_info_.back();
    end_lane_way_point_->mutable_pose()->set_x(last_point.x);
    end_lane_way_point_->mutable_pose()->set_y(last_point.y);
    end_lane_way_point_->set_heading(std::atan2(last_point.dy, last_point.dx));
  }
  return true;
}

bool PathFollowMap::GetRouteSegments(
    const common::VehicleState &vehicle_state,
    std::list<apollo::hdmap::RouteSegments> *const route_segments) {
  CHECK_NOTNULL(route_segments);
  if (nullptr == lane_info_ || extra_center_info_.empty()) {
    return false;
  }
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  // Find the point on curve mapped to vehicle position.
  double adc_x = vehicle_state.x();
  double adc_y = vehicle_state.y();
  size_t mapping_point_index = FindMapingPointOnLane(adc_x, adc_y);
  double mapping_point_s = extra_center_info_[mapping_point_index].s;
  last_curve_index_of_vehicle_ = mapping_point_index;
  double start_s = mapping_point_s - look_backward_distance;
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  double end_s = mapping_point_s + look_forward_distance;
  if (end_s > extra_center_info_.back().s) {
    end_s = extra_center_info_.back().s;
  }
  route_segments->clear();
  route_segments->emplace_back();
  route_segments->back().emplace_back(
      apollo::hdmap::LaneSegment(lane_info_, start_s, end_s));
  auto &last_route_segment = route_segments->back();
  if (mapping_point_index == extra_center_info_.size()) {
    last_route_segment.SetRouteEndWaypoint(last_route_segment.LastWaypoint());
  }
  last_route_segment.SetCanExit(false);
  last_route_segment.SetNextAction(apollo::routing::FORWARD);
  last_route_segment.SetId("0_0");
  last_route_segment.SetStopForDestination(true);
  last_route_segment.SetIsOnSegment(true);
  last_route_segment.SetPreviousAction(apollo::routing::FORWARD);
  return true;
}

bool PathFollowMap::ExtendSegments(
    const apollo::hdmap::RouteSegments &segments, double start_s, double end_s,
    apollo::hdmap::RouteSegments *const truncated_segments) const {
  truncated_segments->clear();
  if (nullptr == lane_info_ || extra_center_info_.empty()) {
    return false;
  }
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  bool is_reach_end = false;
  if (end_s >= extra_center_info_.back().s) {
    end_s = extra_center_info_.back().s;
    is_reach_end = true;
  }
  truncated_segments->emplace_back(
      apollo::hdmap::LaneSegment(lane_info_, start_s, end_s));
  if (is_reach_end) {
    truncated_segments->SetRouteEndWaypoint(truncated_segments->LastWaypoint());
  }
  truncated_segments->SetCanExit(false);
  truncated_segments->SetNextAction(apollo::routing::FORWARD);
  truncated_segments->SetId("0_0");
  truncated_segments->SetStopForDestination(true);
  truncated_segments->SetIsOnSegment(true);
  truncated_segments->SetPreviousAction(apollo::routing::FORWARD);
  return true;
}

std::vector<routing::LaneWaypoint> PathFollowMap::FutureRouteWaypoints() const {
  // There is no routing way point, so return empty.
  std::vector<routing::LaneWaypoint> way_points;
  return way_points;
}

bool PathFollowMap::IsValid(const planning::PlanningCommand &command) const {
  external_command::PathFollowCommand path_follow_command;
  if (!GetPathFollowCommand(command, &path_follow_command)) {
    return false;
  }
  return (path_follow_command.way_point().size() > 1);
}

void PathFollowMap::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  end_point = end_lane_way_point_;
}

bool PathFollowMap::GetPathFollowCommand(
    const planning::PlanningCommand &input_command,
    external_command::PathFollowCommand *output_command) {
  if (input_command.has_custom_command()) {
    if (input_command.custom_command()
            .Is<external_command::PathFollowCommand>()) {
      if (input_command.custom_command().UnpackTo(output_command)) {
        return true;
      }
    }
  }
  return false;
}

void PathFollowMap::GetCurve(
    const ::google::protobuf::RepeatedPtrField<apollo::external_command::Point>
        &points,
    bool is_center_curve, apollo::hdmap::Curve *output_curve) {
  output_curve->Clear();
  if (points.size() < 1) {
    return;
  }
  auto curve_segement = output_curve->add_segment();
  auto line_segment = curve_segement->mutable_line_segment();
  // Set s coordinate of start position.
  curve_segement->set_s(0.0);
  // Get curve points from the input points.
  ::apollo::common::PointENU *last_point = nullptr;
  float dx = 0.0;
  float dy = 0.0;
  double distance = 0.0;
  double delta_distance = 0.0;
  const float distance_eps = 0.01;
  const float min_distance_for_calc_heading = 0.2;
  bool is_heading_calc = false;
  for (const auto input_point : points) {
    if (nullptr != last_point) {
      dx = input_point.x() - last_point->x();
      dy = input_point.y() - last_point->y();
      if (std::abs(dx) > distance_eps || std::abs(dy) > distance_eps) {
        delta_distance = std::sqrt(dx * dx + dy * dy);
        distance += delta_distance;
        auto point_enu = line_segment->add_point();
        point_enu->set_x(input_point.x());
        point_enu->set_y(input_point.y());
        if (is_center_curve) {
          extra_center_info_.emplace_back(CurveInfo(distance, input_point.x(),
                                                    input_point.y(),
                                                    delta_distance, dx, dy));
        }
        last_point = point_enu;
      } else {
        continue;
      }
      if (!is_heading_calc && distance > min_distance_for_calc_heading) {
        curve_segement->set_heading(std::atan2(dy, dx));
        is_heading_calc = true;
      }
    } else {
      auto point_enu = line_segment->add_point();
      point_enu->set_x(input_point.x());
      point_enu->set_y(input_point.y());
      if (is_center_curve) {
        extra_center_info_.emplace_back(CurveInfo(
            distance, input_point.x(), input_point.y(), 0.0, 0.0, 0.0));
      }
      last_point = point_enu;
      curve_segement->mutable_start_position()->set_x(point_enu->x());
      curve_segement->mutable_start_position()->set_y(point_enu->y());
    }
  }
  // Modify (dx, dy) of the start point with the second point, which will be
  // used to calculate the heading.
  if (extra_center_info_.size() > 1) {
    extra_center_info_[0].s = extra_center_info_[1].s;
    extra_center_info_[0].ds = extra_center_info_[1].ds;
    extra_center_info_[0].dx = extra_center_info_[1].dx;
    extra_center_info_[0].dy = extra_center_info_[1].dy;
  }
  curve_segement->set_length(distance);
}

void PathFollowMap::GetBoundary(
    const ::google::protobuf::RepeatedPtrField<apollo::external_command::Point>
        &boundary_points,
    apollo::hdmap::LaneBoundary *output_boundary) {
  GetCurve(boundary_points, false, output_boundary->mutable_curve());
  if (output_boundary->curve().segment().size() > 0) {
    output_boundary->set_length(
        output_boundary->curve().segment().Get(0).length());
  }
  output_boundary->set_virtual_(false);
}

void PathFollowMap::GetBoundary(
    const apollo::hdmap::Curve &center_curve, double left_path_width,
    double right_path_width, apollo::hdmap::LaneBoundary *left_boundary,
    apollo::hdmap::LaneBoundary *right_boundary) const {
  if (center_curve.segment().size() < 1) {
    return;
  }
  const auto &line_segment = center_curve.segment().Get(0).line_segment();
  auto &point_list = line_segment.point();
  if (point_list.size() < 2) {
    return;
  }
  CHECK_EQ(point_list.size(), extra_center_info_.size());
  auto info = extra_center_info_.begin();
  double unit_direction_x = 0.0;
  double unit_direction_y = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  auto left_curve = left_boundary->mutable_curve();
  auto left_curve_segement = left_curve->add_segment();
  auto left_line_segment = left_curve_segement->mutable_line_segment();
  left_curve_segement->set_s(0.0);
  ::apollo::common::PointENU *last_left_point = nullptr;
  double left_length = 0.0;
  auto right_curve = right_boundary->mutable_curve();
  auto right_curve_segement = right_curve->add_segment();
  auto right_line_segment = right_curve_segement->mutable_line_segment();
  right_curve_segement->set_s(0.0);
  ::apollo::common::PointENU *last_right_point = nullptr;
  double right_length = 0.0;
  for (auto point = point_list.begin(); point != point_list.end();
       ++point, ++info) {
    // Calculate heading of the point and generate the left and right boundary
    // along the perpendicular direction of heading.
    unit_direction_x = info->dx / info->ds;
    unit_direction_y = info->dy / info->ds;
    // Deal with point on left boundary.
    auto left_point = left_line_segment->add_point();
    left_point->set_x(point->x() + unit_direction_x * left_path_width);
    left_point->set_y(point->y() + unit_direction_y * left_path_width);
    if (nullptr != last_left_point) {
      dx = left_point->x() - last_left_point->x();
      dy = left_point->y() - last_left_point->y();
      left_length += std::sqrt(dx * dx + dy * dy);
    }
    last_left_point = left_point;
    // Deal with point on right boundary.
    auto right_point = right_line_segment->add_point();
    right_point->set_x(point->x() + unit_direction_x * right_path_width);
    right_point->set_y(point->y() + unit_direction_y * right_path_width);
    if (nullptr != last_right_point) {
      dx = right_point->x() - last_right_point->x();
      dy = right_point->y() - last_right_point->y();
      right_length += std::sqrt(dx * dx + dy * dy);
    }
    last_right_point = right_point;
  }
  // Set left boundary.
  const auto &first_segment = center_curve.segment().Get(0);
  left_curve_segement->set_s(first_segment.s());
  left_curve_segement->mutable_start_position()->CopyFrom(
      left_line_segment->point().Get(0));
  left_curve_segement->set_heading(first_segment.heading());
  left_curve_segement->set_length(left_length);
  left_boundary->set_length(left_length);
  left_boundary->set_virtual_(true);
  // Set right boundary.
  right_curve_segement->set_s(first_segment.s());
  right_curve_segement->mutable_start_position()->CopyFrom(
      right_line_segment->point().Get(0));
  right_curve_segement->set_heading(first_segment.heading());
  right_curve_segement->set_length(right_length);
  right_boundary->set_length(right_length);
  right_boundary->set_virtual_(true);
}

size_t PathFollowMap::FindMapingPointOnLane(double adc_x, double adc_y) {
  double vec_x = 0.0;
  double vec_y = 0.0;
  double dot = 0.0;
  size_t i = last_curve_index_of_vehicle_;
  for (; i < extra_center_info_.size(); ++i) {
    const auto &center_info = extra_center_info_[i];
    vec_x = adc_x - center_info.x;
    vec_y = adc_y - center_info.y;
    dot = vec_x * center_info.dx + vec_y * center_info.y;
    if (dot < 0.0) {
      return i;
    }
  }
  return i;
}

}  // namespace planning
}  // namespace apollo
