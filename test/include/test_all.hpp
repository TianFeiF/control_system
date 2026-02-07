#pragma once

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "ecrt.h"
#ifdef __cplusplus
}
#endif


#define SUCCESS 0
#define FAILURE -1

#define CYCLE_US 8000  // 4ms 周期
#define BusAlias 0

#define ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS  0
#define ETHERCAT_SLAVE_1_HCFA_X3E_POS       1
#define ETHERCAT_SLAVE_2_HCFA_X3E_POS       2
#define ETHERCAT_SLAVE_3_HCFA_X3E_POS       3
#define ETHERCAT_SLAVE_4_HANS_ROBOT_POS     4
#define ETHERCAT_SLAVE_5_HANS_ROBOT_POS     5
#define ETHERCAT_SLAVE_6_HANS_ROBOT_POS     6
#define ETHERCAT_SLAVE_7_F2838x_POS         7

#define INEXBOT_IO_R4_VENDOR_ID 0x00000025
#define INEXBOT_IO_R4_PRODUCT_CODE 0x00000530
#define INEXBOT_IO_R4_REVISION_NUMBER 0x00010000

// #define HCFA_X3E_VENDOR_ID 0x000116c7
// #define HCFA_X3E_PRODUCT_CODE 0x003e0402
// #define HCFA_X3E_REVISION_NUMBER 0x00000001

#define HCFA_X3E_VENDOR_ID 0x000116c7
#define HCFA_X3E_PRODUCT_CODE 0x005e0402
#define HCFA_X3E_REVISION_NUMBER 0x00000001

#define HANS_ROBOT_VENDOR_ID 0x0000001a
#define HANS_ROBOT_PRODUCT_CODE 0x50440200
#define HANS_ROBOT_REVISION_NUMBER 0x05132016

#define F2838x_VENDOR_ID 0x00201911
#define F2838x_PRODUCT_CODE 0x10003201
#define F2838x_REVISION_NUMBER 0x00000001

#define SLAVE_COUNT 8

#define INEXBOT_IO_R4_ID 1
#define F2838x_DEVICE_ID 2

#define SETUP 1
#define SETDOWN 0

/*
 * ======================================================================================
 * 全局变量定义
 * ======================================================================================
 */

/* EtherCAT 主站实例指针 */
extern ec_master_t *master;

/* EtherCAT 域 (Domain) 指针
 * Domain 用于管理一组 PDO 的数据交换。本例中使用一个 Domain (domain1) 管理所有从站的 PDO。
 */
extern ec_domain_t *domain1;

/* 
 * 域数据指针 (Process Data Pointer)
 * 指向 Domain 映射的内存区域。读写 PDO 数据时，通过 offset 偏移量在此内存区域操作。
 * 例如：EC_READ_U16(domain1_pd + offset)
 */
extern uint8_t *domain1_pd;

/* 
 * 周期唤醒时间点
 * 用于 sleep_until 函数，确保主循环以精确的周期运行。
 */
extern struct timespec wakeup_time;

/* 从站配置对象数组 */
//static ec_slave_config_t *slave_configs[SLAVE_COUNT] = {};
/* 从站物理位置数组（别名/位置） */
//static unsigned int slave_positions[SLAVE_COUNT] = {};
/* 实际配置的从站数量 */
//static unsigned int slave_count = 0;

/* 运行标志位，用于信号处理和退出循环 */
//static volatile int run = 1;

typedef struct {
    bool active;
    uint16_t slave_idx;     // Physical slave index
    //ma_axis_type_t type;
    
    // Scaling
    double scale_pos;
    double scale_vel;
    
    // State Machine / Logic State
    bool servo_enabled;
    int32_t last_actual_pos;
    uint32_t time_cnt;
    int32_t csp_target;
    int csp_warmup;
    uint8_t fault_reset_cycles;

    bool cmd_run;
    int cmd_dir;
    int cmd_step;

    struct {
        unsigned int controlWord;
        unsigned int workModeOut;
        unsigned int targetPosition;
        unsigned int targetTorque;
        unsigned int touchProbeFunc;
        unsigned int interpolationCtrl;
    } out;

    struct {
        unsigned int statusword;
        unsigned int workModeIn;
        unsigned int actualPosition;
        unsigned int actualVelocity;
        unsigned int actualTorque;
        unsigned int errorCode;
        unsigned int followingError;
        unsigned int digitalInputs;
        unsigned int touchProbeStatus;
        unsigned int touchProbePos;
        unsigned int servoErrorCode;
        unsigned int brakeDelay;
        unsigned int accelerometer;
        unsigned int multiPositionActualValue;
    } in;
    
    // For IO driver
    struct {
        unsigned int output_offset; // RxPDO offset
        unsigned int input_offset;  // TxPDO offset
        unsigned int dac_output_ch[2];
        unsigned int adc_input_ch[2];
        uint8_t size_out;           // bytes
        uint8_t size_in;            // bytes
    } io;

    // Axis Configuration (Loaded from JSON)
    struct {
        int axis_id;
        char name[64];
        char description[128];
        
        struct {
            int encoder_resolution_bits;
            char encoder_type[16];
            double gear_ratio;
            double screw_pitch_mm;
            double unit_per_rev;
            int rotation_direction;
        } mechanical;

        struct {
            double max_velocity_units;
            double max_acceleration_units;
            double max_deceleration_units;
            double max_jerk_units;
            double default_velocity_units;
        } kinematics;

        struct {
            double soft_limit_pos_units;
            double soft_limit_neg_units;
            double max_position_error_units;
            double max_current_amp;
        } limits;

        struct {
            int method;
            double speed_fast_units;
            double speed_slow_units;
            double offset_units;
            double current_threshold_amp;
        } homing;

        struct {
            bool is_synced;
            int sync_master_axis_id;
            char sync_type[16];
            int sync_direction;
        } synchronization;
    } config;

} slave_data;

extern slave_data device_io;
extern slave_data device_hcfa_servo[3];
extern slave_data device_hans_robot[3][2];
extern slave_data device_f2838x;

extern const ec_pdo_entry_reg_t domain1_regs[];

extern ec_pdo_entry_info_t slave_0_pdo_entries[];
extern ec_pdo_info_t slave_0_pdos[];
extern ec_sync_info_t slave_0_syncs[];
extern ec_pdo_entry_info_t slave_1_pdo_entries[];
extern ec_pdo_info_t slave_1_pdos[];
extern ec_sync_info_t slave_1_syncs[];
extern ec_pdo_entry_info_t slave_2_pdo_entries[];
extern ec_pdo_info_t slave_2_pdos[];
extern ec_sync_info_t slave_2_syncs[];
extern ec_pdo_entry_info_t slave_3_pdo_entries[];
extern ec_pdo_info_t slave_3_pdos[];
extern ec_sync_info_t slave_3_syncs[];
extern ec_pdo_entry_info_t slave_4_pdo_entries[];
extern ec_pdo_info_t slave_4_pdos[];
extern ec_sync_info_t slave_4_syncs[];
extern ec_pdo_entry_info_t slave_5_pdo_entries[];
extern ec_pdo_info_t slave_5_pdos[];
extern ec_sync_info_t slave_5_syncs[];
extern ec_pdo_entry_info_t slave_6_pdo_entries[];
extern ec_pdo_info_t slave_6_pdos[];
extern ec_sync_info_t slave_6_syncs[];
extern ec_pdo_entry_info_t slave_7_pdo_entries[];
extern ec_pdo_info_t slave_7_pdos[];
extern ec_sync_info_t slave_7_syncs[];
