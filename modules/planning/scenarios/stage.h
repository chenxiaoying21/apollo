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

/**
 * @file
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/planning/proto/scenario_pipeline.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class Stage {
 public:
  enum StageStatus {
    ERROR = 1,
    READY = 2,
    RUNNING = 3,
    FINISHED = 4,
  };

  Stage();

  virtual bool Init(const StagePipeline& config,
                    const std::shared_ptr<DependencyInjector>& injector,
                    const std::string& config_dir, void* context);

  virtual ~Stage() = default;

  /**
   * @brief Each stage does its business logic inside Process function.
   * If the stage want to transit to a different stage after finish,
   * it should set the type of 'next_stage_'.
   */
  virtual StageStatus Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief The sequence of tasks inside the stage. These tasks usually will be
   * executed in order.
   */
  const std::vector<std::shared_ptr<Task>>& TaskList() const {
    return task_list_;
  }

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() {
    return static_cast<T*>(context_);
  }

  Task* FindTask(const std::string& task_type) const;

  const std::string& NextStage() const { return next_stage_; }

 protected:
  bool ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnReferenceLineForOnlineLearning(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnOpenSpace(Frame* frame);

  virtual Stage::StageStatus FinishScenario();

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

 protected:
  std::map<std::string, std::shared_ptr<Task>> tasks_;
  std::vector<std::shared_ptr<Task>> task_list_;
  std::string next_stage_;
  void* context_;
  std::shared_ptr<DependencyInjector> injector_;
  StagePipeline pipeline_config_;

 private:
  std::string name_;
};

}  // namespace planning
}  // namespace apollo
