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

#include <map>
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"

namespace apollo {
namespace cyber {
namespace plugin_manager {

class PluginManager {
 public:
  ~PluginManager();

  /**
   * @brief parse plugin description file and load the library
   *
   * @param file_path the path of plugin description file
   * @return process result, true for success
   */
  bool ProcessPluginDescriptionFile(const std::string& file_path);

  /**
   * @brief load plugin clases from file
   *
   * @param pluin_description_file_path file path
   * @return result of loadding plugin, true for success
   */
  bool LoadPlugin(const std::string& plugin_description_file_path);

  /**
   * @brief get singleton instance of PluginManager
   *
   * @return instance pointer
   */
  static PluginManager* Instance();

  /**
   * @brief create plugin instance of derived class based on `Base`
   *
   * @param derived_class class name of the derived class
   * @return instance pointer
   */
  template <typename Base>
  std::shared_ptr<Base> CreateInstance(const std::string& derived_class);

 private:
  apollo::cyber::class_loader::ClassLoaderManager class_loader_manager_;

  static PluginManager* instance_;
};

template <typename Base>
std::shared_ptr<Base> PluginManager::CreateInstance(
    const std::string& derived_class) {
  AINFO << "creating plugin instance of " << derived_class;
  return class_loader_manager_.CreateClassObj<Base>(derived_class);
}

#define CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(name, base) \
  CLASS_LOADER_REGISTER_CLASS(name, base)

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
