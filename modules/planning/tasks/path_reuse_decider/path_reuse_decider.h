/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/tasks/path_reuse_decider/proto/path_reuse_decider.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/tasks/common/decider.h"

namespace apollo {
namespace planning {

class PathReuseDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  void GetCurrentStopPositions(
      Frame* frame,
      std::vector<const common::PointENU*>* current_stop_positions);

  // get current s_projection of history objects which has stop decisions
  void GetHistoryStopPositions(
      ReferenceLineInfo* const reference_line_info,
      const std::vector<const HistoryObjectDecision*>&
          history_objects_decisions,
      std::vector<std::pair<const double, const common::PointENU>>*
          history_stop_positions);

  void GetADCSLPoint(const ReferenceLine& reference_line,
                     common::SLPoint* adc_position_sl);

  bool GetBlockingObstacleS(ReferenceLineInfo* const reference_line_info,
                            double* blocking_obstacle_s);
  // ignore blocking obstacle when it is far away
  bool IsIgnoredBlockingObstacle(ReferenceLineInfo* const reference_line_info);
  // check if path is collision free
  bool IsCollisionFree(ReferenceLineInfo* const reference_line_info);

  // check path length
  bool NotShortPath(const DiscretizedPath& current_path);

  // trim history path
  bool TrimHistoryPath(Frame* frame,
                       ReferenceLineInfo* const reference_line_info);

 private:
  static int reusable_path_counter_;  // count reused path
  static int total_path_counter_;     // count total path
  static bool path_reusable_;         // if the path is reusable
  PathReuseDeciderConfig config_;     // the config the task
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathReuseDecider, Task)

}  // namespace planning
}  // namespace apollo
