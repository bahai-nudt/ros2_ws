#pragma once

#include <string>

#include "config/global_config.h"

namespace msf {

/** 从 YAML 解析全部小节到 GlobalConfig（不含 calib.json）。 */
bool LoadGlobalConfigFromYaml(const std::string& config_file, GlobalConfig& config);

/** YAML + calib.json 完整加载全局配置（main 只需调用这一次）。 */
bool LoadGlobalConfig(const std::string& config_file, GlobalConfig& config);

/** 从 calib.json 加载外参与杆臂（io 内部，LoadGlobalConfig 调用）。 */
bool LoadCalib(const std::string& calib_path, GlobalConfig& config);

}  // namespace msf
