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
 */

#pragma once

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/tools/scroll_recorder/proto/scroll_recorder_conf.pb.h"

namespace apollo {
namespace cyber {
namespace record {
class Recorder;
}
}  // namespace cyber
}  // namespace apollo

/**
 * @namespace apollo::scroll_recorder
 * @brief apollo::scroll_recorder
 */
namespace apollo {
namespace scroll_recorder {

class ScrollRecorderComponent : public apollo::cyber::TimerComponent {
 public:
  ScrollRecorderComponent();

  bool Init() override;

  bool Proc() override;

 private:
  double GetDuration(const std::string& record_file_path);

  void StartRecord();

  static void SignalHandle(int sig);

  apollo::scroll_recorder::ScrollRecorderConf scroll_recorder_conf_;
  double scroll_time_;
  std::string bag_directory_;
  std::vector<std::string> record_files_;
  std::map<std::string, double> file_duration_map_;
  static std::shared_ptr<cyber::record::Recorder> recorder_;
};

CYBER_REGISTER_COMPONENT(ScrollRecorderComponent)

}  // namespace scroll_recorder
}  // namespace apollo
