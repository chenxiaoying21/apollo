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

#include "modules/planning/tasks/path_bounds_decider/path_bounds_decider.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class PathBoundsDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_is_lane_borrowing(false);
    injector_ = std::make_shared<DependencyInjector>();
  }

  virtual void TearDown() {}

 protected:
  PathBoundsDeciderConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

TEST_F(PathBoundsDeciderTest, Init) {
  PathBoundsDecider path_bounds_decider;
  path_bounds_decider.Init(
      "scenarios/lane_follow_scenario/conf/lane_follow_stage",
      "PATH_BOUNDS_DECIDER", injector_);
  EXPECT_EQ(path_bounds_decider.Name(), "PATH_BOUNDS_DECIDER");
}

}  // namespace planning
}  // namespace apollo
