#pragma once

#include <cstdint>

/**
 * EtherCAT 初始化函数
 * 
 * 功能：
 * 1. 请求 EtherCAT 主站实例
 * 2. 创建 Domain (Process Data Domain)
 * 3. 配置从站 (PDO 映射, DC 同步)
 * 4. 注册 PDO Entry
 * 5. 激活主站并获取 Process Data 内存指针
 * 
 * @return 0 on success, -1 on failure
 */
int init_ethercat();

/**
 * DC 时钟同步预热函数
 * 
 * 功能：
 * 在正式使能伺服前，运行一段“空循环”，仅进行过程数据交换和时钟同步。
 * 
 * @param cycles 预热周期数
 * @return 0 on success, -1 on failure
 */
int sync_clocks(int cycles);

/**
 * 轴预备函数 (CSP 模式切换)
 * 
 * 功能：
 * 1. 将伺服控制模式切换为 CSP (Mode 8)
 * 2. 将目标位置初始化为当前实际位置
 * 3. 等待从站反馈 Modes of Operation Display == 8
 * 
 * @return 0 on success, -1 on failure
 */
int prepare_all_axes();

/**
 * 单轴使能函数
 * 
 * 功能：
 * 执行 CiA402 状态机序列使能单个轴。
 * 
 * @param axis_id 轴ID (1-9)
 * @return 0 on success, -1 on failure
 */
int enable_axis(int axis_id);

/**
 * 多轴使能函数
 * 
 * 功能：
 * 并行控制所有轴执行 CiA402 状态机切换序列，使其进入 Operation Enabled 状态。
 * 
 * @return 0 on success, -1 on failure
 */
int enable_all_axes();

/**
 * 设置轴目标位置
 * 
 * @param axis_id 轴ID (1-9)
 * @param target 目标位置
 * @return 0 on success, -1 on failure
 */
int set_axis_targetpos(int axis_id, int32_t target);

/**
 * 设置 IO 端口状态
 * 
 * @param io_id IO设备ID (1: INEXBOT_IO_R4_ID, 2: F2838x_DEVICE_ID)
 * @param port_index 端口索引 (0-15)
 * @param state true=高电平, false=低电平
 * @return 0 on success, -1 on failure
 */
int set_io_state(int io_id, int port_index, bool state);

/**
 * 设置 ADC 输出值
 * 
 * @param io_id IO设备ID (1: INEXBOT_IO_R4_ID, 2: F2838x_DEVICE_ID)
 * @param channel 通道索引 (0-1)
 * @param value 输出值 (16位)
 * @return 0 on success, -1 on failure
 */
int set_adc_value(int io_id, int channel, uint16_t value);


