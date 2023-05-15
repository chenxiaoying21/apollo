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

#pragma once

#include <memory>

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/tasks/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class PathGeneration : public Task {
 public:
  virtual ~PathGeneration() = default;

  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  apollo::common::Status Execute(Frame* frame) override;

 protected:
  virtual apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) {
    return apollo::common::Status::OK();
  }

  virtual apollo::common::Status Process(Frame* frame) {
    return apollo::common::Status::OK();
  }
  /**
   * @brief calculate init sl state by planning start point, result will store
   * in init_sl_state_
   */
  void GetStartPointSLState();
  void RecordDebugInfo(const PathBound& path_boundaries,
                       const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);

  void RecordDebugInfo(const PathData& path_data, const std::string& debug_name,
                       ReferenceLineInfo* const reference_line_info);
  SLState init_sl_state_;
};

}  // namespace planning
}  // namespace apollo
