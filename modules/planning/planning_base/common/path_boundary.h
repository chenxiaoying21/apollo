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

#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace apollo {
namespace planning {

class PathBoundary {
 public:
  PathBoundary(const double start_s, const double delta_s,
               std::vector<std::pair<double, double>> path_boundary);
  /**
   * @brief construct class by path bound
   * @param delta_s The distance between two point in s-axis
   * @param path_bound tuple first is s, second is l_min, third is l_max
   *    **/
  PathBoundary(
      const double delta_s,
      const std::vector<std::tuple<double, double, double>>& path_bound);
  virtual ~PathBoundary() = default;

  double start_s() const;

  double delta_s() const;

  void set_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& boundary() const;

  void set_label(const std::string& label);
  const std::string& label() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;

 private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::vector<std::pair<double, double>> boundary_;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
};

}  // namespace planning
}  // namespace apollo
