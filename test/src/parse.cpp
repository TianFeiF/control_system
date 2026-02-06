#include "test_all.hpp"
// 引入 nlohmann/json 库，这是 C++ 中非常流行的单头文件 JSON 库
// 使用前需确保 json.hpp 已在 include 路径中
#include "nlohmann/json.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// 使用别名简化代码，之后可以直接用 json 代替 nlohmann::json
using json = nlohmann::json;

/**
 * @brief 安全获取 JSON 字段值的辅助函数模板
 * 
 * nlohmann/json 库基础用法说明：
 * 1. j.contains(key): 检查对象中是否存在指定的键，返回 bool。
 * 2. j[key].is_null(): 检查值是否为 null。
 * 3. j[key].get<T>(): 将 JSON 值转换为 C++ 类型 T。如果类型不匹配会抛出异常。
 * 
 * @tparam T 目标类型 (如 int, double, bool)
 * @param j JSON 对象
 * @param key 键名
 * @param target 目标变量引用
 */
template<typename T>
void get_optional(const json& j, const char* key, T& target) {
    // 1. 先检查键是否存在 (contains)
    // 2. 再检查值是否不为 null (!is_null)
    if (j.contains(key) && !j[key].is_null()) {
        try {
            // 尝试将值转换为目标类型 T 并赋值
            target = j[key].get<T>();
        } catch (const json::exception& e) {
            // 如果类型转换失败 (例如 JSON 中是字符串但目标是 int)，捕获异常并打印警告
            std::cerr << "Error parsing key '" << key << "': " << e.what() << std::endl;
        }
    }
}

/**
 * @brief 针对 C 风格字符串 (char数组) 的特化版本
 * 
 * @param j JSON 对象
 * @param key 键名
 * @param target 目标 char 数组
 * @param max_len 数组最大长度
 */
void get_optional(const json& j, const char* key, char* target, size_t max_len) {
    // 检查键是否存在且值为字符串类型 (.is_string())
    if (j.contains(key) && j[key].is_string()) {
        // 先获取为 std::string
        std::string s = j[key].get<std::string>();
        // 安全拷贝到 char 数组，防止溢出
        strncpy(target, s.c_str(), max_len - 1);
        target[max_len - 1] = '\0'; // 确保以 null 结尾
    }
}

/**
 * @brief 解析单个轴的 JSON 配置并填充到 slave_data 结构体
 * 
 * @param slave 目标结构体指针
 * @param axis_json 单个轴的 JSON 对象
 */
void populate_axis_config(slave_data* slave, const json& axis_json) {
    // 解析基础信息
    get_optional(axis_json, "axis_id", slave->config.axis_id);
    get_optional(axis_json, "name", slave->config.name, sizeof(slave->config.name));
    get_optional(axis_json, "description", slave->config.description, sizeof(slave->config.description));

    // 解析 "mechanical" 对象
    // 使用 .contains() 检查是否存在子对象
    if (axis_json.contains("mechanical")) {
        // 获取子对象引用，避免拷贝
        const auto& m = axis_json["mechanical"];
        get_optional(m, "encoder_resolution_bits", slave->config.mechanical.encoder_resolution_bits);
        get_optional(m, "encoder_type", slave->config.mechanical.encoder_type, sizeof(slave->config.mechanical.encoder_type));
        get_optional(m, "gear_ratio", slave->config.mechanical.gear_ratio);
        get_optional(m, "screw_pitch_mm", slave->config.mechanical.screw_pitch_mm);
        get_optional(m, "unit_per_rev", slave->config.mechanical.unit_per_rev);
        get_optional(m, "rotation_direction", slave->config.mechanical.rotation_direction);
    }

    // 解析 "kinematics" 对象
    if (axis_json.contains("kinematics")) {
        const auto& k = axis_json["kinematics"];
        get_optional(k, "max_velocity_units", slave->config.kinematics.max_velocity_units);
        get_optional(k, "max_acceleration_units", slave->config.kinematics.max_acceleration_units);
        get_optional(k, "max_deceleration_units", slave->config.kinematics.max_deceleration_units);
        get_optional(k, "max_jerk_units", slave->config.kinematics.max_jerk_units);
        get_optional(k, "default_velocity_units", slave->config.kinematics.default_velocity_units);
    }

    // 解析 "limits" 对象
    if (axis_json.contains("limits")) {
        const auto& l = axis_json["limits"];
        get_optional(l, "soft_limit_pos_units", slave->config.limits.soft_limit_pos_units);
        get_optional(l, "soft_limit_neg_units", slave->config.limits.soft_limit_neg_units);
        get_optional(l, "max_position_error_units", slave->config.limits.max_position_error_units);
        get_optional(l, "max_current_amp", slave->config.limits.max_current_amp);
    }

    // 解析 "homing" 对象
    if (axis_json.contains("homing")) {
        const auto& h = axis_json["homing"];
        get_optional(h, "method", slave->config.homing.method);
        get_optional(h, "speed_fast_units", slave->config.homing.speed_fast_units);
        get_optional(h, "speed_slow_units", slave->config.homing.speed_slow_units);
        get_optional(h, "offset_units", slave->config.homing.offset_units);
        get_optional(h, "current_threshold_amp", slave->config.homing.current_threshold_amp);
    }

    // 解析 "synchronization" 对象
    if (axis_json.contains("synchronization")) {
        const auto& s = axis_json["synchronization"];
        get_optional(s, "is_synced", slave->config.synchronization.is_synced);
        get_optional(s, "sync_master_axis_id", slave->config.synchronization.sync_master_axis_id);
        get_optional(s, "sync_type", slave->config.synchronization.sync_type, sizeof(slave->config.synchronization.sync_type));
        get_optional(s, "sync_direction", slave->config.synchronization.sync_direction);
    }
}

/**
 * @brief 主解析函数：从文件加载配置并填充数组
 * 
 * @param config_path 配置文件路径 (json)
 * @param slaves 目标 slave_data 数组
 * @param max_slaves 数组最大容量，防止越界
 * @return int 0 成功, -1 失败
 */
int load_axis_config(const char* config_path, slave_data* slaves, int max_slaves) {
    // 1. 打开文件流
    std::ifstream file(config_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << config_path << std::endl;
        return -1;
    }

    try {
        // 2. 解析 JSON
        // nlohmann/json 支持直接从流中读取并解析
        // 语法: file >> root; 就像读取 cin 一样简单
        json root;
        file >> root;

        // 3. 验证根结构
        // .is_object(): 检查根是否为对象 {}
        // .is_array(): 检查 "axes" 是否为数组 []
        if (!root.is_object() || !root.contains("axes") || !root["axes"].is_array()) {
            std::cerr << "Invalid JSON structure: 'axes' array not found" << std::endl;
            return -1;
        }

        // 4. 遍历数组
        // 使用 range-based for loop 遍历 json 数组
        // axis_val 是数组中的每个元素 (也是 json 对象)
        for (const auto& axis_val : root["axes"]) {
            if (!axis_val.is_object()) continue;
            
            // 获取 axis_id 用于索引
            int axis_id = 0;
            if (axis_val.contains("axis_id")) {
                axis_id = axis_val["axis_id"].get<int>();
            }

            // 映射 axis_id 到数组索引 (axis_id 从 1 开始 -> index 从 0 开始)
            if (axis_id > 0 && axis_id <= max_slaves) {
                // 调用填充函数
                populate_axis_config(&slaves[axis_id - 1], axis_val);
            } else {
                std::cerr << "Warning: Axis ID " << axis_id << " out of range (1-" << max_slaves << ")" << std::endl;
            }
        }
    } catch (const json::parse_error& e) {
        // 专门捕获 JSON 格式错误 (如漏了逗号、大括号不匹配)
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        // 捕获其他潜在异常
        std::cerr << "Error processing config: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
