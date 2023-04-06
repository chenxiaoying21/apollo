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

#include <tinyxml2.h>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace plugin_manager {

PluginManager::~PluginManager() {}

bool PluginManager::ProcessPluginDescriptionFile(const std::string& file_path) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    AWARN << "fail to process file " << file_path;
    return false;
  }
  const tinyxml2::XMLElement* root = doc.RootElement();

  std::string library_path = root->Attribute("path");
  // TODO(liangjinping): resolve relative library path
  class_loader_manager_.LoadLibrary(library_path);

  // TODO(liangjinping): parse description file and do something more

  return true;
}

bool PluginManager::LoadPlugin(
    const std::string& plugin_description_file_path) {
  AINFO << "loading plugin from " << plugin_description_file_path;
  return ProcessPluginDescriptionFile(plugin_description_file_path);
}

PluginManager* PluginManager::Instance() { return instance_; }

PluginManager* PluginManager::instance_ = new PluginManager;

}  // namespace plugin_manager
}  // namespace cyber
}  // namespace apollo
