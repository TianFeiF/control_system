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

#include "ecrt.h"







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