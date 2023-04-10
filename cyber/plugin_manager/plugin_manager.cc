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

#include "cyber/plugin_manager/plugin_manager.h"

#include <dirent.h>
#include <unistd.h>

#include <memory>
#include <regex>
#include <vector>

#include <tinyxml2.h>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_description.h"

namespace apollo {
namespace cyber {
namespace plugin_manager {

PluginManager::~PluginManager() {}

bool PluginManager::ProcessPluginDescriptionFile(const std::string& file_path,
                                                 std::string* library_path) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    AWARN << "fail to process file " << file_path;
    return false;
  }
  const tinyxml2::XMLElement* root = doc.RootElement();

  // TODO(liangjinping): parse as struct
  *library_path = root->Attribute("path");

  // TODO(liangjinping): parse description file and do something more

  return true;
}

/**
 * @brief resolve library path
 * @param library_path path of library
 * @param is_sys_first check apollo distribution home path first
 * @return resolved path, return original path if path absolute or not exists
 */
std::string ResolveLibraryPath(const std::string& library_path,
                               const bool is_sys_first = false) {
  if (library_path[0] == '/') {
    return std::string(library_path);
  }
  // TODO(liangjinping): change to configurable
  const std::string cwd = apollo::cyber::common::GetCurrentPath();
  const std::string bazel_bin_path = cwd + "/bazel-bin";
  const std::string usr_plugin_library_path =
      bazel_bin_path + "/" + library_path;

  const std::string apollo_distribution_home =
      apollo::cyber::common::GetEnv("APOLLO_DISTRIBUTION_HOME");
  const std::string sys_plugin_library_path =
      apollo_distribution_home + "/lib/plugins/" + library_path;

  if (is_sys_first &&
      apollo::cyber::common::PathExists(sys_plugin_library_path)) {
    return sys_plugin_library_path;
  } else if (apollo::cyber::common::PathExists(usr_plugin_library_path)) {
    return usr_plugin_library_path;
  } else if (apollo::cyber::common::PathExists(sys_plugin_library_path)) {
    return sys_plugin_library_path;
  }
  // path invalid, return original path
  return std::string(library_path);
}

bool PluginManager::LoadPlugin(
    const std::string& plugin_description_file_path) {
  AINFO << "loading plugin from " << plugin_description_file_path;
  // load plugin without name, use library path as name

  std::string library_path;
  if (!ProcessPluginDescriptionFile(plugin_description_file_path,
                                    &library_path)) {
    // invalid description file
    AWARN << "invalid description file " << plugin_description_file_path;
    return false;
  }
  std::string plugin_name =
      std::regex_replace(library_path, std::regex("/"), std::string("__"));
  // try to detect the real path
  library_path = ResolveLibraryPath(library_path);
  if (!apollo::cyber::common::PathExists(library_path)) {
    AWARN << "load plugin " << plugin_description_file_path
          << " failed, library " << library_path << " not found";
    return false;
  }
  if (!class_loader_manager_.LoadLibrary(library_path)) {
    AWARN << "load plugin " << plugin_description_file_path
          << " failed, library " << library_path << " invalid";
    return false;
  }
  plugin_description_map_[plugin_name] =
      std::make_shared<PluginDescription>(PluginDescription(
          plugin_name, "", plugin_description_file_path, library_path));
  AINFO << "load plugin " << plugin_name << " " << plugin_description_file_path
        << " success";
  return true;
}

/**
 * @brief find cyber_plugin_index directory
 * @param base_path search root
 * @param path_list vector for storing result
 * @return true if at least one is found
 */
bool FindPlunginIndexPath(const std::string& base_path,
                          std::vector<std::string>* path_list) {
  // TODO(liangjinping): change to configurable
  size_t count = apollo::cyber::common::FindPathByPattern(
      base_path, "cyber_plugin_index", DT_DIR, true, path_list);
  return count > 0;
}

bool PluginManager::FindPluginIndexAndLoad(const std::string& plugin_index_path,
                                           const bool is_sys) {
  std::vector<std::string> plugin_index_list;
  apollo::cyber::common::FindPathByPattern(plugin_index_path, "", DT_REG, false,
                                           &plugin_index_list);
  const std::string cwd = apollo::cyber::common::GetCurrentPath();
  const std::string apollo_distribution_home =
      apollo::cyber::common::GetEnv("APOLLO_DISTRIBUTION_HOME");
  bool success = true;
  for (auto plugin_index : plugin_index_list) {
    AINFO << "plugin index found " << plugin_index;
    std::string plugin_description_file_path;
    std::string plugin_name =
        apollo::cyber::common::GetFileName(plugin_description_file_path);
    if (apollo::cyber::common::GetContent(plugin_index,
                                          &plugin_description_file_path)) {
      if (plugin_description_file_path[0] != '/') {
        if (is_sys) {
          plugin_description_file_path =
              apollo_distribution_home + "/" + plugin_description_file_path;
        } else {
          // find in code workspace
          plugin_description_file_path =
              cwd + "/" + plugin_description_file_path;
        }
      }
      AINFO << "loading plugin from " << plugin_description_file_path;
      if (apollo::cyber::common::PathExists(plugin_description_file_path)) {
        std::string library_path;
        if (!ProcessPluginDescriptionFile(plugin_description_file_path,
                                          &library_path)) {
          success = false;
          // invalid description file
          AWARN << "invalid description file " << plugin_description_file_path;
          continue;
        }
        // try to detect the real path
        library_path = ResolveLibraryPath(library_path, is_sys);
        if (!apollo::cyber::common::PathExists(library_path)) {
          success = false;
          AWARN << "load plugin " << plugin_description_file_path
                << " failed, library " << library_path << " not found";
          continue;
        }
        if (!class_loader_manager_.LoadLibrary(library_path)) {
          success = false;
          AWARN << "load plugin " << plugin_name << " failed, library "
                << library_path << "invalid";
          continue;
        }
        plugin_description_map_[plugin_name] =
            std::make_shared<PluginDescription>(
                PluginDescription(plugin_name, plugin_index,
                                  plugin_description_file_path, library_path));
        AINFO << "load plugin " << plugin_name << " "
              << plugin_description_file_path << " success";
      }
    }
  }
  return success;
}

bool PluginManager::LoadInstalledPlugins() {
  const std::string cwd = apollo::cyber::common::GetCurrentPath();
  const std::string bazel_bin_path = cwd + "/bazel-bin";

  std::vector<std::string> user_plugin_index_path_list;
  AINFO << "scanning user plugin index path";
  FindPlunginIndexPath(bazel_bin_path, &user_plugin_index_path_list);
  // load user plugin with higher priority
  for (auto dir : user_plugin_index_path_list) {
    AINFO << "load user plugin index path: " << dir;
    FindPluginIndexAndLoad(dir, false);
  }

  const std::string apollo_distribution_home =
      apollo::cyber::common::GetEnv("APOLLO_DISTRIBUTION_HOME");
  // TODO(liangjinping): change to configurable
  const std::string system_plugin_index_path =
      apollo_distribution_home + "/share/cyber_plugin_index";
  AINFO << "load system plugin index path: " << system_plugin_index_path;
  FindPluginIndexAndLoad(system_plugin_index_path, true);
  return true;
}

PluginManager* PluginManager::Instance() { return instance_; }

PluginManager* PluginManager::instance_ = new PluginManager;

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
