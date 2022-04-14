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
#include "modules/tools/scroll_recorder/scroll_recorder_component.h"

#include <regex>

#include "cyber/common/file.h"
#include "cyber/common/time_conversion.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/tools/cyber_recorder/recorder.h"

using Time = ::apollo::cyber::Time;

namespace apollo {
namespace scroll_recorder {

std::shared_ptr<cyber::record::Recorder> ScrollRecorderComponent::recorder_ =
    NULL;

bool compare_string(const std::string& a, const std::string& b) {
  return a > b;
}

ScrollRecorderComponent::ScrollRecorderComponent()
    : scroll_time_(600), bag_directory_("") {}

bool ScrollRecorderComponent::Init() {
  if (!GetProtoConfig(&scroll_recorder_conf_)) {
    AERROR << "Unable to load scroll recorder conf file: " << ConfigFilePath();
    return false;
  }

  // Create the bag directory for scroll recorder.
  bag_directory_ = scroll_recorder_conf_.bag_directory();
  if (!cyber::common::EnsureDirectory(bag_directory_)) {
    AERROR << "Failed to create the bag directory!";
    return false;
  }
  if ('/' != bag_directory_[bag_directory_.size() - 1]) {
    bag_directory_ += "/";
  }
  StartRecord();
  AINFO << "Record is starting and data will be saved into " << bag_directory_
        << ".";
  signal(SIGTERM, ScrollRecorderComponent::SignalHandle);
  scroll_time_ = scroll_recorder_conf_.scroll_time();
  record_files_.clear();
  file_duration_map_.clear();
  return true;
}

bool ScrollRecorderComponent::Proc() {
  // Get all the record files.
  std::vector<std::string> files =
      cyber::common::ListSubPaths(bag_directory_, DT_REG);
  std::smatch result;
  std::regex record_file_name_regex("[1-9][0-9]{13}\\.record\\.[0-9]{5}");
  for (const auto& file : files) {
    if (std::regex_match(file, result, record_file_name_regex)) {
      if (std::find(record_files_.begin(), record_files_.end(), file) ==
          record_files_.end()) {
        record_files_.emplace_back(file);
      }
      double duration = GetDuration(bag_directory_ + file);
      file_duration_map_[file] = duration;
    }
  }
  // Sort the files in name order.
  std::sort(record_files_.begin(), record_files_.end(), compare_string);
  // Delete the overdue files.
  double total_time = 0.0;
  double file_duration = 0.0;
  for (auto iter = record_files_.begin(); iter != record_files_.end();) {
    file_duration = file_duration_map_[*iter];
    if (total_time > scroll_time_) {
      file_duration_map_.erase(*iter);
      if (0 != std::remove((bag_directory_ + (*iter)).c_str())) {
        AWARN << "Failed to delete file: " << *iter;
      }
      iter = record_files_.erase(iter);
      if (record_files_.end() == iter) {
        break;
      }
    } else {
      ++iter;
    }
    total_time += file_duration;
  }
  return true;
}

double ScrollRecorderComponent::GetDuration(
    const std::string& record_file_path) {
  cyber::record::RecordFileReader file_reader;
  if (!file_reader.Open(record_file_path)) {
    AERROR << "open record file error. file: " << record_file_path;
    return 0.0;
  }

  cyber::proto::Header hdr = file_reader.GetHeader();
  auto begin_time_s = static_cast<double>(hdr.begin_time()) / 1e9;
  auto end_time_s = static_cast<double>(hdr.end_time()) / 1e9;
  auto duration_s = end_time_s - begin_time_s;
  file_reader.Close();
  return duration_s;
}

void ScrollRecorderComponent::StartRecord() {
  std::vector<std::string> opt_output_vec;
  std::vector<std::string> opt_white_channels;
  std::vector<std::string> opt_black_channels;
  std::string default_output_file =
      bag_directory_ +
      cyber::common::UnixSecondsToString(time(nullptr), "%Y%m%d%H%M%S") +
      ".record";
  opt_output_vec.push_back(default_output_file);
  // ::apollo::cyber::Init(argv[0]);
  bool opt_all = (TopicSpecifiedMode::SPECIFY_EXCLUDE_ONLY ==
                  scroll_recorder_conf_.topic_specified_mode());
  if (opt_all) {
    for (const auto& t : scroll_recorder_conf_.topic()) {
      opt_black_channels.emplace_back(t);
    }
  } else {
    for (const auto& t : scroll_recorder_conf_.topic()) {
      opt_white_channels.emplace_back(t);
    }
  }
  auto opt_header = cyber::record::HeaderBuilder::GetHeader();
  recorder_ = std::make_shared<cyber::record::Recorder>(
      opt_output_vec[0], opt_all, opt_white_channels, opt_black_channels,
      opt_header);
  recorder_->Start();
}

void ScrollRecorderComponent::SignalHandle(int sig) {
  recorder_->Stop();
  exit(0);
}

}  // namespace scroll_recorder
}  // namespace apollo
