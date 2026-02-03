#include "test_all.hpp"
#include <cstdio>
#include <atomic>
#include <pthread.h>
#include <termios.h>
#include <sys/select.h>
#include <fcntl.h>
#include <cmath>

/*
 * 这是一个最小化的 EtherCAT 主站测试程序，用于：
 *  - 初始化 IgH EtherCAT Master（ecrt 接口）
 *  - 完成从站 PDO 映射与 domain 注册，获取 process data 指针（domain1_pd）
 *  - 进行分布式时钟（DC）同步预热，避免在伺服使能前时钟未锁定导致状态机异常
 *  - 以 CiA402（0x6040/0x6041）状态机序列使能伺服，并在使能前写入 CSP 模式（mode=8）
 *  - 进入循环周期收发 process data，示例性读写 IO/位置等 PDO
 *
 * 设计约定：
 *  - 所有 PDO 的“偏移量 offset”都来自 [test_all.hpp] 中的 domain1_regs 注册结果。
 *    读写 process data 时通过 domain1_pd + offset 访问。
 *  - 本文件只做“测试用途”的流程串联，未实现完整的伺服控制策略。
 */

ec_master_t *master = NULL;
ec_domain_t *domain1 = NULL;
ec_slave_config_t *sc = NULL;
uint8_t *domain1_pd = NULL;
struct timespec wakeup_time;

static ec_slave_config_t *slave_configs[SLAVE_COUNT] = {};
static unsigned int slave_positions[SLAVE_COUNT] = {};
static unsigned int slave_count = 0;

static volatile int run = 1;
static std::atomic<int> g_selected_axis{1};
static std::atomic<int> g_axis_dir{0};

void sleep_until(struct timespec *ts, long delay_us);
static uint64_t monotonic_time_ns();
static void print_process_table_if_needed(const ec_master_state_t *ms, const ec_domain_state_t *ds);

static void print_master_domain_state(const char *tag)
{
    ec_master_state_t ms = {};
    ec_domain_state_t ds = {};
    ecrt_master_state(master, &ms);
    ecrt_domain_state(domain1, &ds);
    fprintf(stderr,
            "[%s] master: slaves_responding=%u al_states=0x%02X link_up=%u\n",
            tag,
            ms.slaves_responding,
            ms.al_states,
            ms.link_up);
    fprintf(stderr,
            "[%s] domain: wc_state=%u wc=%u redundancy_active=%u\n",
            tag,
            ds.wc_state,
            ds.working_counter,
            ds.redundancy_active);
}

static void print_slave_states(const char *tag)
{
    for (unsigned int i = 0; i < slave_count; ++i) {
        if (!slave_configs[i]) {
            continue;
        }
        ec_slave_config_state_t ss = {};
        ecrt_slave_config_state(slave_configs[i], &ss);
        fprintf(stderr,
                "[%s] slave(pos=%u): online=%u operational=%u al_state=0x%02X\n",
                tag,
                slave_positions[i],
                ss.online,
                ss.operational,
                ss.al_state);
    }
}

static slave_data *axis_device(int axis_id)
{
    if (axis_id >= 1 && axis_id <= 3) {
        return &device_hcfa_servo[axis_id - 1];
    }
    if (axis_id >= 4 && axis_id <= 9) {
        switch (axis_id) {
        case 4:
            return &device_hans_robot[0][0];
        case 5:
            return &device_hans_robot[0][1];
        case 6:
            return &device_hans_robot[1][0];
        case 7:
            return &device_hans_robot[1][1];
        case 8:
            return &device_hans_robot[2][0];
        case 9:
            return &device_hans_robot[2][1];
        default:
            return NULL;
        }
    }
    return NULL;
}

static const char *wc_state_str(ec_wc_state_t s)
{
    switch (s) {
    case EC_WC_ZERO:
        return "ZERO";
    case EC_WC_INCOMPLETE:
        return "INC";
    case EC_WC_COMPLETE:
        return "OK";
    default:
        return "UNK";
    }
}

static void *keyboard_thread_main(void *)
{
    struct termios orig = {};
    if (tcgetattr(STDIN_FILENO, &orig) != 0) {
        while (run) {
            usleep(1000);
        }
        return NULL;
    }

    struct termios raw = orig;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
    }

    int esc_state = 0;
    while (run) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        const int rv = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (rv <= 0) {
            continue;
        }

        unsigned char buf[16];
        const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const unsigned char c = buf[i];
            if (esc_state == 1) {
                if (c == '[') {
                    esc_state = 2;
                } else {
                    esc_state = 0;
                }
                continue;
            }
            if (esc_state == 2) {
                if (c == 'D') {
                    g_axis_dir.store(-1);
                } else if (c == 'C') {
                    g_axis_dir.store(+1);
                } else if (c == 'A') {
                    g_axis_dir.store(+1);
                } else if (c == 'B') {
                    g_axis_dir.store(-1);
                }
                esc_state = 0;
                continue;
            }

            if (c >= '1' && c <= '9') {
                g_selected_axis.store((int)(c - '0'));
                continue;
            }
            if (c == '0') {
                g_selected_axis.store(0);
                g_axis_dir.store(0);
                continue;
            }
            if (c == ' ' || c == 's' || c == 'S') {
                g_axis_dir.store(0);
                continue;
            }
            if (c == 'q' || c == 'Q') {
                run = 0;
                break;
            }

            if (c == 0x1b) {
                esc_state = 1;
                continue;
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    if (old_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }
    return NULL;
}

static int wait_for_domain_wc(unsigned int min_wc, int stable_cycles, int timeout_ms)
{
    if ((int)min_wc <= 0) {
        return 0;
    }
    if (stable_cycles <= 0) {
        stable_cycles = 1;
    }
    if (timeout_ms <= 0) {
        timeout_ms = 1;
    }

    const int max_cycles = (int) (timeout_ms * 1000 / CYCLE_US);
    int stable = 0;
    for (int cycle = 0; cycle < max_cycles && run; ++cycle) {
        sleep_until(&wakeup_time, CYCLE_US);

        ecrt_master_application_time(master, monotonic_time_ns());

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);

        ec_master_state_t ms = {};
        ec_domain_state_t ds = {};
        ecrt_master_state(master, &ms);
        ecrt_domain_state(domain1, &ds);
        print_process_table_if_needed(&ms, &ds);

        if (ms.link_up && ds.wc_state == EC_WC_COMPLETE && ds.working_counter >= min_wc) {
            stable++;
        } else {
            stable = 0;
        }
        if (stable >= stable_cycles) {
            return 0;
        }

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }

    ec_master_state_t ms = {};
    ec_domain_state_t ds = {};
    ecrt_master_state(master, &ms);
    ecrt_domain_state(domain1, &ds);
    fprintf(stderr,
            "wait_for_domain_wc timeout: wc=%u wc_state=%u link_up=%u al_states=0x%02X (min_wc=%u stable_cycles=%d timeout_ms=%d)\n",
            ds.working_counter,
            ds.wc_state,
            ms.link_up,
            ms.al_states,
            min_wc,
            stable_cycles,
            timeout_ms);
    print_master_domain_state("wait_wc_timeout");
    print_slave_states("wait_wc_timeout");
    return -1;
}

static void print_process_table_if_needed(const ec_master_state_t *ms, const ec_domain_state_t *ds)
{
    static unsigned long long cycle_count = 0;
    static int is_tty = -1;
    const unsigned long long print_div = 10;

    if ((cycle_count % print_div) != 0) {
        cycle_count++;
        return;
    }

    if (is_tty < 0) {
        is_tty = isatty(fileno(stdout)) ? 1 : 0;
    }

    if (is_tty) {
        printf("\033[H\033[J");
    } else {
        static unsigned long long header_countdown = 0;
        if (header_countdown == 0) {
            printf("+--------+----+-------+------+-----+----------+----------+----------+------------+------------+------------+------------+------------+------------+------------+------------+------------+\n");
            printf("| cycle  | wc | wc_st | link | al  | in16     | adc_in1  | adc_in2  | pos1       | pos2       | pos3       | pos4       | pos5       | pos6       | pos7       | pos8       | pos9       |\n");
            printf("+--------+----+-------+------+-----+----------+----------+----------+------------+------------+------------+------------+------------+------------+------------+------------+------------+\n");
            header_countdown = 200;
        }
        header_countdown--;
    }

    const uint32_t input_val_5 = EC_READ_U16(domain1_pd + device_io.io.input_offset);
    const uint32_t input_val_6 = EC_READ_U16(domain1_pd + device_io.io.adc_input_ch[0]);
    const uint32_t input_val_7 = EC_READ_U16(domain1_pd + device_io.io.adc_input_ch[1]);
    const uint32_t input_val_8 = EC_READ_U32(domain1_pd + device_hcfa_servo[0].in.actualPosition);
    const uint32_t input_val_9 = EC_READ_U32(domain1_pd + device_hcfa_servo[1].in.actualPosition);
    const uint32_t input_val_10 = EC_READ_U32(domain1_pd + device_hcfa_servo[2].in.actualPosition);
    const uint32_t input_val_11 = EC_READ_U32(domain1_pd + device_hans_robot[0][0].in.actualPosition);
    const uint32_t input_val_12 = EC_READ_U32(domain1_pd + device_hans_robot[0][1].in.actualPosition);
    const uint32_t input_val_13 = EC_READ_U32(domain1_pd + device_hans_robot[1][0].in.actualPosition);
    const uint32_t input_val_14 = EC_READ_U32(domain1_pd + device_hans_robot[1][1].in.actualPosition);
    const uint32_t input_val_15 = EC_READ_U32(domain1_pd + device_hans_robot[2][0].in.actualPosition);
    const uint32_t input_val_16 = EC_READ_U32(domain1_pd + device_hans_robot[2][1].in.actualPosition);

    const uint32_t input_val_17 = EC_READ_U32(domain1_pd + device_hcfa_servo[0].out.targetPosition);
    const uint32_t input_val_18 = EC_READ_U32(domain1_pd + device_hcfa_servo[1].out.targetPosition);
    const uint32_t input_val_19 = EC_READ_U32(domain1_pd + device_hcfa_servo[2].out.targetPosition);
    const uint32_t input_val_20 = EC_READ_U32(domain1_pd + device_hans_robot[0][0].out.targetPosition);
    const uint32_t input_val_21 = EC_READ_U32(domain1_pd + device_hans_robot[0][1].out.targetPosition);
    const uint32_t input_val_22 = EC_READ_U32(domain1_pd + device_hans_robot[1][0].out.targetPosition);
    const uint32_t input_val_23 = EC_READ_U32(domain1_pd + device_hans_robot[1][1].out.targetPosition);
    const uint32_t input_val_24 = EC_READ_U32(domain1_pd + device_hans_robot[2][0].out.targetPosition);
    const uint32_t input_val_25 = EC_READ_U32(domain1_pd + device_hans_robot[2][1].out.targetPosition);

    printf("+--------+----+-------+------+-----+----------+----------+----------+------------+------------+------------+------------+------------+------------+------------+------------+------------+\n");
    printf("| row    | wc | wc_st | link | al  | in16     | adc_in1  | adc_in2  | p1         | p2         | p3         | p4         | p5         | p6         | p7         | p8         | p9         |\n");
    printf("+--------+----+-------+------+-----+----------+----------+----------+------------+------------+------------+------------+------------+------------+------------+------------+------------+\n");
    printf("| A%5llu | %2u | %5s | %4u | %03X | 0x%04X   | 0x%04X   | 0x%04X   | %10d | %10d | %10d | %10d | %10d | %10d | %10d | %10d | %10d |\n",
           (unsigned long long)cycle_count,
           ds->working_counter,
           wc_state_str(ds->wc_state),
           ms->link_up,
           ms->al_states,
           (unsigned)input_val_5 & 0xFFFFu,
           (unsigned)input_val_6 & 0xFFFFu,
           (unsigned)input_val_7 & 0xFFFFu,
           (int32_t)input_val_8,
           (int32_t)input_val_9,
           (int32_t)input_val_10,
           (int32_t)input_val_11,
           (int32_t)input_val_12,
           (int32_t)input_val_13,
           (int32_t)input_val_14,
           (int32_t)input_val_15,
           (int32_t)input_val_16);
    printf("| TGT    | -- | ----- | ---- | --- | -------- | -------- | -------- | %10d | %10d | %10d | %10d | %10d | %10d | %10d | %10d | %10d |\n",
           (int32_t)input_val_17,
           (int32_t)input_val_18,
           (int32_t)input_val_19,
           (int32_t)input_val_20,
           (int32_t)input_val_21,
           (int32_t)input_val_22,
           (int32_t)input_val_23,
           (int32_t)input_val_24,
           (int32_t)input_val_25);
    printf("+--------+----+-------+------+-----+----------+----------+----------+------------+------------+------------+------------+------------+------------+------------+------------+------------+\n");
    fflush(stdout);

    cycle_count++;
}


int init_ethercat ()
{

    printf("Requesting EtherCAT master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }

    printf("Creating domain...\n");
    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }

    /*
     * slave_specs 定义“需要配置 PDO 的从站清单”，每个条目包含：
     *  - position: 站号（基于 BusAlias 的物理位置）
     *  - vendor_id/product_code: ESI 识别信息
     *  - syncs: 该从站的 PDO/SyncMgr 描述（见 test_all.hpp）
     *
     * 这里的循环会逐个调用：
     *  - ecrt_master_slave_config: 获取从站配置句柄 sc_i
     *  - ecrt_slave_config_pdos: 把 syncs 中描述的 PDO 映射写入 master 配置
     *
     * 注意：这里只配置 PDO 映射，不在此处做 SDO 参数写入（如果需要可扩展）。
     */
    struct {
        unsigned int position;
        uint32_t vendor_id;
        uint32_t product_code;
        const ec_sync_info_t *syncs;
    } slave_specs[] = {
        {ETHERCAT_SLAVE_0_INEXBOT_IO_R4_POS, INEXBOT_IO_R4_VENDOR_ID, INEXBOT_IO_R4_PRODUCT_CODE, slave_0_syncs},
        {ETHERCAT_SLAVE_1_HCFA_X3E_POS, HCFA_X3E_VENDOR_ID, HCFA_X3E_PRODUCT_CODE, slave_1_syncs},
        {ETHERCAT_SLAVE_2_HCFA_X3E_POS, HCFA_X3E_VENDOR_ID, HCFA_X3E_PRODUCT_CODE, slave_2_syncs},
        {ETHERCAT_SLAVE_3_HCFA_X3E_POS, HCFA_X3E_VENDOR_ID, HCFA_X3E_PRODUCT_CODE, slave_3_syncs},
        {ETHERCAT_SLAVE_4_HANS_ROBOT_POS, HANS_ROBOT_VENDOR_ID, HANS_ROBOT_PRODUCT_CODE, slave_4_syncs},
        {ETHERCAT_SLAVE_5_HANS_ROBOT_POS, HANS_ROBOT_VENDOR_ID, HANS_ROBOT_PRODUCT_CODE, slave_5_syncs},
        {ETHERCAT_SLAVE_6_HANS_ROBOT_POS, HANS_ROBOT_VENDOR_ID, HANS_ROBOT_PRODUCT_CODE, slave_6_syncs},
        {ETHERCAT_SLAVE_7_F2838x_POS, F2838x_VENDOR_ID, F2838x_PRODUCT_CODE, slave_7_syncs},
    };

    for (unsigned int i = 0; i < (sizeof(slave_specs) / sizeof(slave_specs[0])); ++i) {
        printf("Configuring slave %u...\n", i);
        ec_slave_config_t *sc_i = ecrt_master_slave_config(
            master,
            BusAlias,
            slave_specs[i].position,
            slave_specs[i].vendor_id,
            slave_specs[i].product_code
        );
        if (!sc_i) {
            fprintf(stderr, "Failed to get slave configuration for slave %u.\n", i);
            return -1;
        }

        if (i < SLAVE_COUNT) {
            slave_configs[i] = sc_i;
            slave_positions[i] = slave_specs[i].position;
        }

        if (ecrt_slave_config_pdos(sc_i, EC_END, slave_specs[i].syncs)) {
            fprintf(stderr, "Failed to configure PDOs for slave %u.\n", i);
            return -1;
        }
    }
    slave_count = (unsigned int)(sizeof(slave_specs) / sizeof(slave_specs[0]));

    printf("Registering PDO entries...\n");
    /*
     * domain1_regs 在 [test_all.hpp] 中定义，负责把每个 PDO entry 注册到 domain，
     * 并把“entry 在 process data 中的偏移量”回写到对应的 device_xxx 结构体里。
     *
     * 例如：device_hcfa_servo[0].out.controlWord 最终会变成一个 offset，
     * 之后通过 EC_WRITE_U16(domain1_pd + offset, value) 写入到 0x6040:00。
     */
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed.\n");
        return -1;
    }

    printf("Activating master...\n");
    /*
     * 激活 master 会完成内部资源申请并固定 PDO 映射。
     * 之后可通过 ecrt_domain_data 拿到 process data 的内存指针。
     */
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "Failed to activate master.\n");
        return -1;
    }

    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) {
        fprintf(stderr, "Failed to retrieve domain data pointer.\n");
        return -1;
    }

    printf("Started.\n");
    printf("wakeup_time: %ld.%09ld\n", wakeup_time.tv_sec, wakeup_time.tv_nsec);
    print_master_domain_state("init");
    print_slave_states("init");
    return 0;
}

void signal_handler(int sig) {
    (void) sig;
    run = 0;
}

/*
 * 简单的周期等待函数（CLOCK_MONOTONIC + 绝对时间睡眠）：
 *  - 外部维护 wakeup_time，循环中按固定周期 CYCLE_US 往前推进
 *  - 使用 TIMER_ABSTIME 避免相对 sleep 累积漂移
 */
void sleep_until(struct timespec *ts, long delay_us) {
    ts->tv_nsec += delay_us * 1000;
    while (ts->tv_nsec >= 1000000000) {
        ts->tv_nsec -= 1000000000;
        ts->tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
}

static uint64_t monotonic_time_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

int sync_clocks(int cycles)
{
    if (cycles <= 0) {
        return 0;
    }

    /*
     * 分布式时钟（DC）同步预热：
     *  - ecrt_master_application_time: 通知 master 当前应用时间（ns）
     *  - ecrt_master_sync_reference_clock: 同步参考时钟（通常为 DC reference）
     *  - ecrt_master_sync_slave_clocks: 同步所有从站时钟
     *
     * 这段循环本质上是在“正常的周期收发流程”里附加 sync 调用，
     * 让总线在进入伺服使能前先稳定一段时间。
     */
    for (int i = 0; i < cycles && run; ++i) {
        sleep_until(&wakeup_time, CYCLE_US);

        ecrt_master_application_time(master, monotonic_time_ns());

        /* 收到主站收包，并解析 domain 的 TxPDO -> domain1_pd */
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        /* DC 同步：参考时钟 + 从站时钟 */
        ecrt_master_sync_reference_clock(master);
        ecrt_master_sync_slave_clocks(master);

        /* 将对 domain1_pd 的写入（RxPDO）排队并发送出去 */
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }

    return run ? 0 : -1;
}

static int prepare_axis(slave_data *dev, int axis_id)
{
    const unsigned int off_target = dev->out.targetPosition;
    const unsigned int off_mode_out = dev->out.workModeOut;
    const unsigned int off_actual = dev->in.actualPosition;
    const unsigned int off_mode_in = dev->in.workModeIn;
    const unsigned int off_status = dev->in.statusword;

    /*
     * prepare_axis 的目标：
     *  1) 将 mode out 置为 CSP（8），并等待 mode in 反馈为 8
     *  2) 将 targetPosition 对齐到 actualPosition（避免使能瞬间产生跟随误差）
     *
     * 如果轴处于 Fault（statusword bit3=1），这里会提前退出，让 enable_axis 负责复位。
     */
    const int max_cycles = (int) (3000 * 1000 / CYCLE_US);
    for (int k = 0; k < max_cycles && run; ++k) {
        sleep_until(&wakeup_time, CYCLE_US);

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        const uint16_t status = EC_READ_U16(domain1_pd + off_status);
        if (status & 0x0008) {
            break;
        }

        /* 用实际位置填充目标位置，降低使能后瞬间 following error 的概率 */
        const int32_t actual_pos = (int32_t) EC_READ_S32(domain1_pd + off_actual);

        EC_WRITE_S32(domain1_pd + off_target, actual_pos);
        EC_WRITE_U8(domain1_pd + off_mode_out, 8);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

        /* 部分驱动需要等待 mode in 生效后再执行 CiA402 使能序列 */
        const uint8_t mode_in = EC_READ_U8(domain1_pd + off_mode_in);
        if (mode_in == 8) {
            return 0;
        }
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    const uint16_t status = EC_READ_U16(domain1_pd + off_status);
    const uint8_t mode_in = EC_READ_U8(domain1_pd + off_mode_in);
    const int32_t actual_pos = (int32_t) EC_READ_S32(domain1_pd + off_actual);
    const int32_t target_pos = (int32_t) EC_READ_S32(domain1_pd + off_target);
    const uint8_t mode_out = EC_READ_U8(domain1_pd + off_mode_out);
    fprintf(stderr,
            "Axis %d prepare failed, status=0x%04X mode_in=%u mode_out=%u actual=%d target=%d offs{status=%u mode_in=%u mode_out=%u actual=%u target=%u}\n",
            axis_id,
            status,
            (unsigned) mode_in,
            (unsigned) mode_out,
            (int) actual_pos,
            (int) target_pos,
            off_status,
            off_mode_in,
            off_mode_out,
            off_actual,
            off_target);
    print_master_domain_state("prepare_fail");
    print_slave_states("prepare_fail");
    return -1;
}

int prepare_all_axes()
{
    struct AxisPrepareCtx {
        int axis_id;
        slave_data *dev;
        unsigned int off_target;
        unsigned int off_mode_out;
        unsigned int off_actual;
        unsigned int off_mode_in;
        unsigned int off_status;
        int done;
    };

    AxisPrepareCtx axes[9] = {};
    int axis_count = 0;
    for (int axis_id = 1; axis_id <= 9; ++axis_id) {
        slave_data *dev = axis_device(axis_id);
        if (!dev) {
            fprintf(stderr, "Invalid axis_id=%d\n", axis_id);
            return -1;
        }
        AxisPrepareCtx ctx = {};
        ctx.axis_id = axis_id;
        ctx.dev = dev;
        ctx.off_target = dev->out.targetPosition;
        ctx.off_mode_out = dev->out.workModeOut;
        ctx.off_actual = dev->in.actualPosition;
        ctx.off_mode_in = dev->in.workModeIn;
        ctx.off_status = dev->in.statusword;
        ctx.done = 0;
        axes[axis_count++] = ctx;
    }

    const int prepare_timeout_ms = 15000;
    const int max_cycles = (int) (prepare_timeout_ms * 1000 / CYCLE_US);
    for (int cycle = 0; cycle < max_cycles && run; ++cycle) {
        sleep_until(&wakeup_time, CYCLE_US);

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        ec_master_state_t ms = {};
        ec_domain_state_t ds = {};
        ecrt_master_state(master, &ms);
        ecrt_domain_state(domain1, &ds);
        print_process_table_if_needed(&ms, &ds);

        for (int i = 0; i < axis_count; ++i) {
            if (axes[i].done) {
                continue;
            }
            const uint16_t status = EC_READ_U16(domain1_pd + axes[i].off_status);
            if (status & 0x0008) {
                continue;
            }

            const int32_t actual_pos = (int32_t) EC_READ_S32(domain1_pd + axes[i].off_actual);
            EC_WRITE_S32(domain1_pd + axes[i].off_target, actual_pos);
            EC_WRITE_U8(domain1_pd + axes[i].off_mode_out, 8);
        }

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

        int done_all = 1;
        for (int i = 0; i < axis_count; ++i) {
            if (axes[i].done) {
                continue;
            }
            const uint8_t mode_in = EC_READ_U8(domain1_pd + axes[i].off_mode_in);
            if (mode_in == 8) {
                axes[i].done = 1;
            } else {
                done_all = 0;
            }
        }
        if (done_all) {
            return 0;
        }
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    for (int i = 0; i < axis_count; ++i) {
        if (axes[i].done) {
            continue;
        }
        const uint16_t status = EC_READ_U16(domain1_pd + axes[i].off_status);
        const uint8_t mode_in = EC_READ_U8(domain1_pd + axes[i].off_mode_in);
        const int32_t actual_pos = (int32_t) EC_READ_S32(domain1_pd + axes[i].off_actual);
        const int32_t target_pos = (int32_t) EC_READ_S32(domain1_pd + axes[i].off_target);
        const uint8_t mode_out = EC_READ_U8(domain1_pd + axes[i].off_mode_out);
        fprintf(stderr,
                "Axis %d prepare timeout, status=0x%04X mode_in=%u mode_out=%u actual=%d target=%d offs{status=%u mode_in=%u mode_out=%u actual=%u target=%u}\n",
                axes[i].axis_id,
                status,
                (unsigned) mode_in,
                (unsigned) mode_out,
                (int) actual_pos,
                (int) target_pos,
                axes[i].off_status,
                axes[i].off_mode_in,
                axes[i].off_mode_out,
                axes[i].off_actual,
                axes[i].off_target);
    }
    print_master_domain_state("prepare_timeout");
    print_slave_states("prepare_timeout");
    return -1;
}

int enable_axis(int axis_id)
{
    slave_data *dev = NULL;
    if (axis_id >= 1 && axis_id <= 3) {
        dev = &device_hcfa_servo[axis_id - 1];
    } else if (axis_id >= 4 && axis_id <= 9) {
        switch (axis_id) {
        case 4:
            dev = &device_hans_robot[0][0];
            break;
        case 5:
            dev = &device_hans_robot[0][1];
            break;
        case 6:
            dev = &device_hans_robot[1][0];
            break;
        case 7:
            dev = &device_hans_robot[1][1];
            break;
        case 8:
            dev = &device_hans_robot[2][0];
            break;
        case 9:
            dev = &device_hans_robot[2][1];
            break;
        default:
            fprintf(stderr, "Invalid axis_id=%d\n", axis_id);
            return -1;
        }
    } else {
        fprintf(stderr, "Invalid axis_id=%d\n", axis_id);
        return -1;
    }

    const unsigned int off_control = dev->out.controlWord;
    const unsigned int off_status = dev->in.statusword;
    const unsigned int off_error = dev->in.errorCode;

    /*
     * CiA402 使能序列（控制字 0x6040 / 状态字 0x6041）：
     *  - 0x06: Shutdown            -> 期待状态 (status&0x006F)==0x0021
     *  - 0x07: Switch on           -> 期待状态 (status&0x006F)==0x0023
     *  - 0x0F: Enable operation    -> 期待状态 (status&0x006F)==0x0027
     *
     * status_mask=0x006F 是常见的“状态机相关位”掩码（CiA402 的低位组合）。
     * 如果 statusword bit3=1（0x0008，Fault），需要先写 0x0080 做 fault reset。
     */
    struct Step {
        uint16_t control;
        uint16_t status_mask;
        uint16_t status_value;
    } steps[] = {
        {0x0006, 0x006F, 0x0021},
        {0x0007, 0x006F, 0x0023},
        {0x000F, 0x006F, 0x0027},
    };

    const int max_cycles_per_step = (int) (3000 * 1000 / CYCLE_US);
    const int max_attempts = 3;
    /*
     * 某些驱动在故障复位/模式切换后需要一定时间才能稳定进入状态机，
     * 因此这里提供多次 attempt，每次 attempt 会先：
     *  - prepare_axis: 对齐目标位置 + 设置 CSP 模式
     *  - fault reset: 若处于 Fault，则循环写 0x0080 直到 fault 位清除
     * 再执行使能步骤序列。
     */
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        if (prepare_axis(dev, axis_id) != 0) {
            fprintf(stderr, "Axis %d enable aborted: prepare failed (attempt=%d)\n", axis_id, attempt + 1);
            return -1;
        }
        for (int k = 0; k < max_cycles_per_step && run; ++k) {
            sleep_until(&wakeup_time, CYCLE_US);

            ecrt_master_receive(master);
            ecrt_domain_process(domain1);

            const uint16_t status = EC_READ_U16(domain1_pd + off_status);
            if (status & 0x0008) {
                /* Fault Reset（bit7=1）：驱动可能要求保持若干周期 */
                EC_WRITE_U16(domain1_pd + off_control, 0x0080);
                ecrt_domain_queue(domain1);
                ecrt_master_send(master);
            } else {
                break;
            }
        }

        int ok_all = 1;
        for (unsigned int s = 0; s < (sizeof(steps) / sizeof(steps[0])); ++s) {
            int ok = 0;
            for (int k = 0; k < max_cycles_per_step && run; ++k) {
                sleep_until(&wakeup_time, CYCLE_US);

                ecrt_master_receive(master);
                ecrt_domain_process(domain1);

                const uint16_t status = EC_READ_U16(domain1_pd + off_status);
                if (status & 0x0008) {
                    ok = 0;
                    break;
                }

                /* 达到该步骤目标状态则进入下一步 */
                if ((status & steps[s].status_mask) == steps[s].status_value) {
                    ok = 1;
                    break;
                }

                /* 写入本步骤控制字，并发送到从站（RxPDO） */
                EC_WRITE_U16(domain1_pd + off_control, steps[s].control);
                ecrt_domain_queue(domain1);
                ecrt_master_send(master);
            }

            if (!ok) {
                ok_all = 0;
                break;
            }
        }

        if (ok_all) {
            return 0;
        }
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    const uint16_t status = EC_READ_U16(domain1_pd + off_status);
    const uint16_t error = EC_READ_U16(domain1_pd + off_error);
    const uint16_t control = EC_READ_U16(domain1_pd + off_control);
    fprintf(stderr,
            "Axis %d enable failed, status=0x%04X error=0x%04X control=0x%04X offs{control=%u status=%u error=%u}\n",
            axis_id,
            status,
            error,
            control,
            off_control,
            off_status,
            off_error);
    print_master_domain_state("enable_fail");
    print_slave_states("enable_fail");
    return -1;

}

int enable_all_axes()
{
    struct AxisEnableCtx {
        int axis_id;
        slave_data *dev;
        unsigned int off_control;
        unsigned int off_status;
        unsigned int off_error;
        int stage;
        int stage_cycles;
        int done;
        int failed;
    };

    AxisEnableCtx axes[9] = {};
    int axis_count = 0;
    for (int axis_id = 1; axis_id <= 9; ++axis_id) {
        slave_data *dev = axis_device(axis_id);
        if (!dev) {
            fprintf(stderr, "Invalid axis_id=%d\n", axis_id);
            return -1;
        }
        AxisEnableCtx ctx = {};
        ctx.axis_id = axis_id;
        ctx.dev = dev;
        ctx.off_control = dev->out.controlWord;
        ctx.off_status = dev->in.statusword;
        ctx.off_error = dev->in.errorCode;
        ctx.stage = 0;
        ctx.stage_cycles = 0;
        ctx.done = 0;
        ctx.failed = 0;
        axes[axis_count++] = ctx;
    }

    struct Step {
        uint16_t control;
        uint16_t status_mask;
        uint16_t status_value;
    } steps[] = {
        {0x0006, 0x006F, 0x0021},
        {0x0007, 0x006F, 0x0023},
        {0x000F, 0x006F, 0x0027},
    };

    const int per_step_timeout_ms = 10000;
    const int fault_reset_timeout_ms = 10000;
    const int per_step_max_cycles = (int) (per_step_timeout_ms * 1000 / CYCLE_US);
    const int fault_reset_max_cycles = (int) (fault_reset_timeout_ms * 1000 / CYCLE_US);
    const int total_timeout_ms = 30000;
    const int total_max_cycles = (int) (total_timeout_ms * 1000 / CYCLE_US);

    for (int cycle = 0; cycle < total_max_cycles && run; ++cycle) {
        sleep_until(&wakeup_time, CYCLE_US);

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        int all_done = 1;
        for (int i = 0; i < axis_count; ++i) {
            if (axes[i].done || axes[i].failed) {
                continue;
            }
            all_done = 0;

            const uint16_t status = EC_READ_U16(domain1_pd + axes[i].off_status);
            const int fault = (status & 0x0008) != 0;

            if (fault) {
                axes[i].stage = -1;
            }

            if (axes[i].stage == -1) {
                EC_WRITE_U16(domain1_pd + axes[i].off_control, 0x0080);
                axes[i].stage_cycles++;
                if (!fault) {
                    axes[i].stage = 0;
                    axes[i].stage_cycles = 0;
                } else if (axes[i].stage_cycles >= fault_reset_max_cycles) {
                    axes[i].failed = 1;
                }
                continue;
            }

            const unsigned int s = (unsigned int) axes[i].stage;
            if (s >= (sizeof(steps) / sizeof(steps[0]))) {
                axes[i].done = 1;
                continue;
            }

            if ((status & steps[s].status_mask) == steps[s].status_value) {
                axes[i].stage++;
                axes[i].stage_cycles = 0;
                if ((unsigned int) axes[i].stage >= (sizeof(steps) / sizeof(steps[0]))) {
                    axes[i].done = 1;
                }
                continue;
            }

            EC_WRITE_U16(domain1_pd + axes[i].off_control, steps[s].control);
            axes[i].stage_cycles++;
            if (axes[i].stage_cycles >= per_step_max_cycles) {
                axes[i].failed = 1;
            }
        }

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

        if (all_done) {
            return 0;
        }
    }

    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    int any_failed = 0;
    for (int i = 0; i < axis_count; ++i) {
        if (axes[i].done && !axes[i].failed) {
            continue;
        }
        any_failed = 1;
        const uint16_t status = EC_READ_U16(domain1_pd + axes[i].off_status);
        const uint16_t error = EC_READ_U16(domain1_pd + axes[i].off_error);
        const uint16_t control = EC_READ_U16(domain1_pd + axes[i].off_control);
        fprintf(stderr,
                "Axis %d enable timeout, status=0x%04X error=0x%04X control=0x%04X stage=%d stage_cycles=%d offs{control=%u status=%u error=%u}\n",
                axes[i].axis_id,
                status,
                error,
                control,
                axes[i].stage,
                axes[i].stage_cycles,
                axes[i].off_control,
                axes[i].off_status,
                axes[i].off_error);
    }
    print_master_domain_state("enable_timeout");
    print_slave_states("enable_timeout");
    return any_failed ? -1 : 0;
}

int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    init_ethercat();
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    pthread_t keyboard_thread;
    if (pthread_create(&keyboard_thread, NULL, keyboard_thread_main, NULL) != 0) {
        fprintf(stderr, "Failed to create keyboard thread.\n");
        run = 0;
        return -1;
    }

    if (sync_clocks(500) != 0) {
        fprintf(stderr, "Clock sync failed.\n");
        run = 0;
        pthread_join(keyboard_thread, NULL);
        return -1;
    }
    if (wait_for_domain_wc(24, 50, 60000) != 0) {
        fprintf(stderr, "Domain wc not ready.\n");
        run = 0;
        pthread_join(keyboard_thread, NULL);
        return -1;
    }
    if (prepare_all_axes() != 0) {
        fprintf(stderr, "Prepare axes failed.\n");
        run = 0;
        pthread_join(keyboard_thread, NULL);
        return -1;
    }
    if (enable_all_axes() != 0) {
        fprintf(stderr, "Enable axes failed.\n");
        run = 0;
        pthread_join(keyboard_thread, NULL);
        return -1;
    }

    int32_t cmd_target[10] = {};
    double cmd_frac[10] = {};
    for (int axis_id = 1; axis_id <= 9; ++axis_id) {
        slave_data *dev = axis_device(axis_id);
        if (!dev) {
            run = 0;
            pthread_join(keyboard_thread, NULL);
            return -1;
        }
        const int32_t actual = (int32_t) EC_READ_S32(domain1_pd + dev->in.actualPosition);
        cmd_target[axis_id] = actual;
        cmd_frac[axis_id] = 0.0;
        EC_WRITE_S32(domain1_pd + dev->out.targetPosition, actual);
    }
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    const double counts_per_rev = 131072.0;
    const double deg_per_sec = 30.0;
    const double counts_per_sec = counts_per_rev * (deg_per_sec / 360.0);
    const double dt_sec = (double) CYCLE_US / 1000000.0;
    const double inc_per_cycle = counts_per_sec * dt_sec;

    while (run) {
        sleep_until(&wakeup_time, CYCLE_US);

        /* 收包 + 解析 TxPDO */
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        ec_master_state_t ms = {};
        ec_domain_state_t ds = {};
        ecrt_master_state(master, &ms);
        ecrt_domain_state(domain1, &ds);
        print_process_table_if_needed(&ms, &ds);

        const int selected_axis = g_selected_axis.load();
        const int axis_dir = g_axis_dir.load();
        if (selected_axis >= 1 && selected_axis <= 9 && axis_dir != 0) {
            cmd_frac[selected_axis] += (double) axis_dir * inc_per_cycle;
            const int32_t delta = (int32_t) cmd_frac[selected_axis];
            if (delta != 0) {
                cmd_target[selected_axis] += delta;
                cmd_frac[selected_axis] -= (double) delta;
            }
        }

        for (int axis_id = 1; axis_id <= 9; ++axis_id) {
            slave_data *dev = axis_device(axis_id);
            if (!dev) {
                run = 0;
                break;
            }
            EC_WRITE_S32(domain1_pd + dev->out.targetPosition, cmd_target[axis_id]);
        }

        /* 示例：写 IO 输出（RxPDO） */
        EC_WRITE_U16(domain1_pd + device_io.io.output_offset, 0xffff);//OUTPUT_1~16
        EC_WRITE_U16(domain1_pd + device_io.io.adc_output_ch[0], 0x07ff);//AD_OUTPUT_1
        EC_WRITE_U16(domain1_pd + device_io.io.adc_output_ch[1], 0x0fff);//AD_OUTPUT_2
        

        //printf("input_val_6: 0x%04X (%.2fV)\n", input_val_6, (float)input_val_6/0x0FFF*10.0);//AD_INPUT_1
        //printf("input_val_7: 0x%04X (%.2fV)\n", input_val_7, (float)input_val_7/0x0FFF*10.0);//AD_INPUT_2
        //printf("input_val_8: 0x%08X\n", input_val_8);//ACTUAL_POSITION_1
        /* 将 RxPDO 写入排队并发送出去 */
        /* 将 RxPDO 写入排队并发送出去 */
        //printf("wakeup_time: %ld.%09ld\n", wakeup_time.tv_sec, wakeup_time.tv_nsec);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
    printf("Releasing master...\n");
    run = 0;
    pthread_join(keyboard_thread, NULL);
    ecrt_release_master(master);
    return 0;
}
