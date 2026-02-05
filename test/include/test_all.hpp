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

#define HCFA_X3E_VENDOR_ID 0x000116c7
#define HCFA_X3E_PRODUCT_CODE 0x003e0402
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

} slave_data;

static slave_data device_io, device_hcfa_servo[3], device_hans_robot[3][2], device_f2838x;


const static ec_pdo_entry_reg_t domain1_regs[] = {
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X7000,6,&device_io.io.output_offset},
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X7000,7,&device_io.io.dac_output_ch[0]},
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X7000,8,&device_io.io.dac_output_ch[1]},
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X6005,0,&device_io.io.input_offset},
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X6006,0,&device_io.io.adc_input_ch[0]},
        {BusAlias,ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS,INEXBOT_IO_R4_VENDOR_ID,INEXBOT_IO_R4_PRODUCT_CODE,0X6007,0,&device_io.io.adc_input_ch[1]},

        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6040,0,&device_hcfa_servo[0].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6060,0,&device_hcfa_servo[0].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X607A,0,&device_hcfa_servo[0].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B8,0,&device_hcfa_servo[0].out.touchProbeFunc},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X603F,0,&device_hcfa_servo[0].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6041,0,&device_hcfa_servo[0].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6064,0,&device_hcfa_servo[0].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6061,0,&device_hcfa_servo[0].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B9,0,&device_hcfa_servo[0].in.touchProbeStatus},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60BA,0,&device_hcfa_servo[0].in.touchProbePos},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60F4,0,&device_hcfa_servo[0].in.followingError},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60FD,0,&device_hcfa_servo[0].in.digitalInputs},
        {BusAlias,ETHERCAT_SLAVE_1_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X213F,0,&device_hcfa_servo[0].in.servoErrorCode},

        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6040,0,&device_hcfa_servo[1].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6060,0,&device_hcfa_servo[1].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X607A,0,&device_hcfa_servo[1].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B8,0,&device_hcfa_servo[1].out.touchProbeFunc},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X603F,0,&device_hcfa_servo[1].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6041,0,&device_hcfa_servo[1].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6064,0,&device_hcfa_servo[1].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6061,0,&device_hcfa_servo[1].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B9,0,&device_hcfa_servo[1].in.touchProbeStatus},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60BA,0,&device_hcfa_servo[1].in.touchProbePos},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60F4,0,&device_hcfa_servo[1].in.followingError},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60FD,0,&device_hcfa_servo[1].in.digitalInputs},
        {BusAlias,ETHERCAT_SLAVE_2_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X213F,0,&device_hcfa_servo[1].in.servoErrorCode},

        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6040,0,&device_hcfa_servo[2].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6060,0,&device_hcfa_servo[2].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X607A,0,&device_hcfa_servo[2].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B8,0,&device_hcfa_servo[2].out.touchProbeFunc},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X603F,0,&device_hcfa_servo[2].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6041,0,&device_hcfa_servo[2].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6064,0,&device_hcfa_servo[2].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X6061,0,&device_hcfa_servo[2].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60B9,0,&device_hcfa_servo[2].in.touchProbeStatus},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60BA,0,&device_hcfa_servo[2].in.touchProbePos},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60F4,0,&device_hcfa_servo[2].in.followingError},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X60FD,0,&device_hcfa_servo[2].in.digitalInputs},
        {BusAlias,ETHERCAT_SLAVE_3_HCFA_X3E_POS,HCFA_X3E_VENDOR_ID,HCFA_X3E_PRODUCT_CODE,0X213F,0,&device_hcfa_servo[2].in.servoErrorCode},

        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6040,0,&device_hans_robot[0][0].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6060,0,&device_hans_robot[0][0].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X607A,0,&device_hans_robot[0][0].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6071,0,&device_hans_robot[0][0].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6840,0,&device_hans_robot[0][1].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6860,0,&device_hans_robot[0][1].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X687A,0,&device_hans_robot[0][1].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6871,0,&device_hans_robot[0][1].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6041,0,&device_hans_robot[0][0].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6077,0,&device_hans_robot[0][0].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6064,0,&device_hans_robot[0][0].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X606C,0,&device_hans_robot[0][0].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X603F,0,&device_hans_robot[0][0].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6061,0,&device_hans_robot[0][0].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3154,0,&device_hans_robot[0][0].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6164,0,&device_hans_robot[0][0].in.multiPositionActualValue},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6841,0,&device_hans_robot[0][1].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6877,0,&device_hans_robot[0][1].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6864,0,&device_hans_robot[0][1].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X686C,0,&device_hans_robot[0][1].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X683F,0,&device_hans_robot[0][1].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6861,0,&device_hans_robot[0][1].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3954,0,&device_hans_robot[0][1].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_4_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6964,0,&device_hans_robot[0][1].in.multiPositionActualValue},

        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6040,0,&device_hans_robot[1][0].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6060,0,&device_hans_robot[1][0].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X607A,0,&device_hans_robot[1][0].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6071,0,&device_hans_robot[1][0].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6840,0,&device_hans_robot[1][1].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6860,0,&device_hans_robot[1][1].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X687A,0,&device_hans_robot[1][1].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6871,0,&device_hans_robot[1][1].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6041,0,&device_hans_robot[1][0].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6077,0,&device_hans_robot[1][0].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6064,0,&device_hans_robot[1][0].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X606C,0,&device_hans_robot[1][0].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X603F,0,&device_hans_robot[1][0].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6061,0,&device_hans_robot[1][0].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3154,0,&device_hans_robot[1][0].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6164,0,&device_hans_robot[1][0].in.multiPositionActualValue},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6841,0,&device_hans_robot[1][1].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6877,0,&device_hans_robot[1][1].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6864,0,&device_hans_robot[1][1].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X686C,0,&device_hans_robot[1][1].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X683F,0,&device_hans_robot[1][1].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6861,0,&device_hans_robot[1][1].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3954,0,&device_hans_robot[1][1].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_5_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6964,0,&device_hans_robot[1][1].in.multiPositionActualValue},

        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6040,0,&device_hans_robot[2][0].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6060,0,&device_hans_robot[2][0].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X607A,0,&device_hans_robot[2][0].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6071,0,&device_hans_robot[2][0].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6840,0,&device_hans_robot[2][1].out.controlWord},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6860,0,&device_hans_robot[2][1].out.workModeOut},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X687A,0,&device_hans_robot[2][1].out.targetPosition},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6871,0,&device_hans_robot[2][1].out.targetTorque},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6041,0,&device_hans_robot[2][0].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6077,0,&device_hans_robot[2][0].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6064,0,&device_hans_robot[2][0].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X606C,0,&device_hans_robot[2][0].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X603F,0,&device_hans_robot[2][0].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6061,0,&device_hans_robot[2][0].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3154,0,&device_hans_robot[2][0].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6164,0,&device_hans_robot[2][0].in.multiPositionActualValue},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6841,0,&device_hans_robot[2][1].in.statusword},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6877,0,&device_hans_robot[2][1].in.actualTorque},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6864,0,&device_hans_robot[2][1].in.actualPosition},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X686C,0,&device_hans_robot[2][1].in.actualVelocity},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X683F,0,&device_hans_robot[2][1].in.errorCode},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6861,0,&device_hans_robot[2][1].in.workModeIn},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X3954,0,&device_hans_robot[2][1].in.accelerometer},
        {BusAlias,ETHERCAT_SLAVE_6_HANS_ROBOT_POS,HANS_ROBOT_VENDOR_ID,HANS_ROBOT_PRODUCT_CODE,0X6964,0,&device_hans_robot[2][1].in.multiPositionActualValue},

        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X7001,1,&device_f2838x.io.output_offset},
        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X7002,1,&device_f2838x.io.dac_output_ch[0]},
        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X7003,1,&device_f2838x.io.dac_output_ch[1]},
        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X6001,1,&device_f2838x.io.input_offset},
        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X6002,1,&device_f2838x.io.adc_input_ch[0]},
        {BusAlias,ETHERCAT_SLAVE_7_F2838x_POS,F2838x_VENDOR_ID,F2838x_PRODUCT_CODE,0X6003,1,&device_f2838x.io.adc_input_ch[1]},
        {},
    };


/* Master 0, Slave 0, "INEXBOT-IO-R4"
 * Vendor ID:       0x00000025
 * Product code:    0x00000530
 * Revision number: 0x00010000
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7000, 0x01, 32}, /* SubIndex 001 */
    {0x7000, 0x02, 32}, /* SubIndex 002 */
    {0x7000, 0x03, 16}, /* SubIndex 003 */
    {0x7000, 0x04, 16}, /* SubIndex 004 */
    {0x7000, 0x05, 32}, /* SubIndex 005 */
    {0x7000, 0x06, 16}, /* SubIndex 006 */
    {0x7000, 0x07, 16}, /* SubIndex 007 */
    {0x7000, 0x08, 16}, /* SubIndex 008 */
    {0x7000, 0x09, 32}, /* SubIndex 009 */
    {0x6000, 0x00, 32}, /* SubIndex 000 */
    {0x6001, 0x00, 32}, /* SubIndex 000 */
    {0x6002, 0x00, 16}, /* SubIndex 000 */
    {0x6003, 0x00, 16}, /* SubIndex 000 */
    {0x6004, 0x00, 32}, /* SubIndex 000 */
    {0x6005, 0x00, 16}, /* SubIndex 000 */
    {0x6006, 0x00, 16}, /* SubIndex 000 */
    {0x6007, 0x00, 16}, /* SubIndex 000 */
    {0x6008, 0x00, 32}, /* SubIndex 000 */
    {0x6009, 0x00, 32}, /* SubIndex 000 */
    {0x600a, 0x00, 32}, /* SubIndex 000 */
    {0x600b, 0x00, 32}, /* SubIndex 000 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 9, slave_0_pdo_entries + 0}, /* RxPDO-Map */
    {0x1a00, 12, slave_0_pdo_entries + 9}, /* TxPDO-Map */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1, "HCFA X3E Servo Driver"
 * Vendor ID:       0x000116c7
 * Product code:    0x003e0402
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* Modes of operation  */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60b8, 0x00, 16}, /* Touch Probe Function */
    {0x603f, 0x00, 16}, /* Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x6061, 0x00, 8}, /* Modes of operation display  */
    {0x60b9, 0x00, 16}, /* Touch Probe Status */
    {0x60ba, 0x00, 32}, /* Touch Probe1 Pos1 Pos Value */
    {0x60f4, 0x00, 32}, /* Following error actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x213f, 0x00, 16}, /* Servo Error Code */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 4, slave_1_pdo_entries + 0}, /* 1st RxPDO-Mapping */
    {0x1a00, 9, slave_1_pdo_entries + 4}, /* 1st TxPDO-Mapping */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2, "HCFA X3E Servo Driver"
 * Vendor ID:       0x000116c7
 * Product code:    0x003e0402
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* Modes of operation  */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60b8, 0x00, 16}, /* Touch Probe Function */
    {0x603f, 0x00, 16}, /* Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x6061, 0x00, 8}, /* Modes of operation display  */
    {0x60b9, 0x00, 16}, /* Touch Probe Status */
    {0x60ba, 0x00, 32}, /* Touch Probe1 Pos1 Pos Value */
    {0x60f4, 0x00, 32}, /* Following error actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x213f, 0x00, 16}, /* Servo Error Code */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 4, slave_2_pdo_entries + 0}, /* 1st RxPDO-Mapping */
    {0x1a00, 9, slave_2_pdo_entries + 4}, /* 1st TxPDO-Mapping */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_2_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_2_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 3, "HCFA X3E Servo Driver"
 * Vendor ID:       0x000116c7
 * Product code:    0x003e0402
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x6060, 0x00, 8}, /* Modes of operation  */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60b8, 0x00, 16}, /* Touch Probe Function */
    {0x603f, 0x00, 16}, /* Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x6061, 0x00, 8}, /* Modes of operation display  */
    {0x60b9, 0x00, 16}, /* Touch Probe Status */
    {0x60ba, 0x00, 32}, /* Touch Probe1 Pos1 Pos Value */
    {0x60f4, 0x00, 32}, /* Following error actual value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
    {0x213f, 0x00, 16}, /* Servo Error Code */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 4, slave_3_pdo_entries + 0}, /* 1st RxPDO-Mapping */
    {0x1a00, 9, slave_3_pdo_entries + 4}, /* 1st TxPDO-Mapping */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_3_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 4, "Hans Robot"
 * Vendor ID:       0x0000001a
 * Product code:    0x50440200
 * Revision number: 0x05132016
 */

ec_pdo_entry_info_t slave_4_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x607a, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x3097, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6840, 0x00, 16},
    {0x6860, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x687a, 0x00, 32},
    {0x6871, 0x00, 16},
    {0x3897, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6041, 0x00, 16},
    {0x6077, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x603f, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3154, 0x00, 16},
    {0x2001, 0x00, 16},
    {0x6164, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6841, 0x00, 16},
    {0x6877, 0x00, 16},
    {0x6864, 0x00, 32},
    {0x686c, 0x00, 32},
    {0x683f, 0x00, 16},
    {0x6861, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3954, 0x00, 16},
    {0x2801, 0x00, 16},
    {0x6964, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
};

ec_pdo_info_t slave_4_pdos[] = {
    {0x1600, 11, slave_4_pdo_entries + 0},
    {0x1610, 11, slave_4_pdo_entries + 11},
    {0x1a00, 12, slave_4_pdo_entries + 22},
    {0x1a10, 12, slave_4_pdo_entries + 34},
};

ec_sync_info_t slave_4_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_4_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_4_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 5, "Hans Robot"
 * Vendor ID:       0x0000001a
 * Product code:    0x50440200
 * Revision number: 0x05132016
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x607a, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x3097, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6840, 0x00, 16},
    {0x6860, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x687a, 0x00, 32},
    {0x6871, 0x00, 16},
    {0x3897, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6041, 0x00, 16},
    {0x6077, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x603f, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3154, 0x00, 16},
    {0x2001, 0x00, 16},
    {0x6164, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6841, 0x00, 16},
    {0x6877, 0x00, 16},
    {0x6864, 0x00, 32},
    {0x686c, 0x00, 32},
    {0x683f, 0x00, 16},
    {0x6861, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3954, 0x00, 16},
    {0x2801, 0x00, 16},
    {0x6964, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1600, 11, slave_5_pdo_entries + 0},
    {0x1610, 11, slave_5_pdo_entries + 11},
    {0x1a00, 12, slave_5_pdo_entries + 22},
    {0x1a10, 12, slave_5_pdo_entries + 34},
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_5_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_5_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 6, "Hans Robot"
 * Vendor ID:       0x0000001a
 * Product code:    0x50440200
 * Revision number: 0x05132016
 */

ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x607a, 0x00, 32},
    {0x6071, 0x00, 16},
    {0x3097, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6840, 0x00, 16},
    {0x6860, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x687a, 0x00, 32},
    {0x6871, 0x00, 16},
    {0x3897, 0x00, 16},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6041, 0x00, 16},
    {0x6077, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x603f, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3154, 0x00, 16},
    {0x2001, 0x00, 16},
    {0x6164, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
    {0x6841, 0x00, 16},
    {0x6877, 0x00, 16},
    {0x6864, 0x00, 32},
    {0x686c, 0x00, 32},
    {0x683f, 0x00, 16},
    {0x6861, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x3954, 0x00, 16},
    {0x2801, 0x00, 16},
    {0x6964, 0x00, 32},
    {0x0000, 0x00, 32}, /* Gap */
    {0x0000, 0x00, 32}, /* Gap */
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1600, 11, slave_6_pdo_entries + 0},
    {0x1610, 11, slave_6_pdo_entries + 11},
    {0x1a00, 12, slave_6_pdo_entries + 22},
    {0x1a10, 12, slave_6_pdo_entries + 34},
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_6_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_6_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 7, "F2838x CPU1 EtherCAT Slave"
 * Vendor ID:       0x00201911
 * Product code:    0x10003201
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_7_pdo_entries[] = {
    {0x7001, 0x01, 16},
    {0x7002, 0x01, 16},
    {0x7003, 0x01, 16},
    {0x7004, 0x01, 16},
    {0x7005, 0x01, 16},
    {0x7006, 0x01, 16},
    {0x7007, 0x01, 16},
    {0x7008, 0x01, 16},
    {0x7009, 0x01, 16},
    {0x700a, 0x01, 16},
    {0x700b, 0x01, 16},
    {0x700c, 0x01, 16},
    {0x700d, 0x01, 16},
    {0x700e, 0x01, 16},
    {0x700f, 0x01, 16},
    {0x7010, 0x01, 16},
    {0x7011, 0x01, 16},
    {0x6001, 0x01, 16}, /* DigitalInputs */
    {0x6002, 0x01, 16}, /* AnalogInputCH1 */
    {0x6003, 0x01, 16}, /* AnalogInputCH2 */
    {0x6004, 0x01, 16}, /* Temperature */
    {0x6005, 0x01, 16}, /* Vdc_Bus */
    {0x6006, 0x01, 16}, /* ModbusState */
    {0x6007, 0x01, 16}, /* ModbusErrorCode */
    {0x6008, 0x01, 16}, /* ModbusRegCount */
    {0x6009, 0x01, 16}, /* ModbusRegData0 */
    {0x600a, 0x01, 16}, /* ModbusRegData1 */
    {0x600b, 0x01, 16}, /* ModbusRegData2 */
    {0x600c, 0x01, 16}, /* ModbusRegData3 */
    {0x600d, 0x01, 16}, /* ModbusRegData4 */
    {0x600e, 0x01, 16}, /* ModbusRegData5 */
    {0x600f, 0x01, 16}, /* ModbusRegData6 */
    {0x6010, 0x01, 16}, /* ModbusRegData7 */
    {0x6011, 0x01, 16}, /* ModbusRegData8 */
    {0x6012, 0x01, 16}, /* ModbusRegData9 */
    {0x6013, 0x01, 16}, /* ModbusRegData10 */
    {0x6014, 0x01, 16}, /* ModbusRegData11 */
};

ec_pdo_info_t slave_7_pdos[] = {
    {0x1600, 17, slave_7_pdo_entries + 0},
    {0x1a00, 20, slave_7_pdo_entries + 17}, /* switches process data mapping */
};

ec_sync_info_t slave_7_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_7_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_7_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
