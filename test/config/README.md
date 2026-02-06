# 轴配置文件说明文档 (Axis Configuration Guide)

本文档详细说明了 `axis_config.json` 配置文件的结构和各字段含义。该文件用于定义运动控制系统中各轴的机械特性、运动学参数、安全限制及多轴同步关系。

## 文件结构

配置文件采用标准的 JSON 格式，根对象包含以下字段：

- `system_description`: 系统描述字符串。
- `version`: 配置文件版本号。
- `axes`: 轴配置数组，每个元素代表一个物理轴或逻辑轴的详细参数。

## 轴参数详解 (Axis Parameters)

每个轴对象包含以下主要分类：

### 1. 基础信息 (Basic Info)
| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `axis_id` | Integer | 轴的唯一标识符，对应 EtherCAT 从站中的轴索引。 |
| `name` | String | 轴的名称（英文），用于日志和调试。 |
| `description` | String | 轴的描述（中文），用于备注用途。 |

### 2. 机械参数 (Mechanical)
描述电机与负载之间的物理连接关系。

| 字段名 | 类型 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `encoder_resolution_bits` | Integer | bits | 编码器分辨率位数（例如 23 表示 $2^{23}$ 脉冲/圈）。 |
| `encoder_type` | String | - | 编码器类型：`absolute` (绝对值) 或 `incremental` (增量式)。 |
| `gear_ratio` | Float | - | 减速比 (电机转速 / 负载转速)。例如 10.0 表示电机转10圈，负载转1圈。 |
| `screw_pitch_mm` | Float | mm | 丝杠导程（仅直线轴有效），旋转一圈移动的距离。 |
| `unit_per_rev` | Float | unit | 负载旋转一圈对应的工程单位量（如 mm 或 degree）。<br>通常计算公式：`screw_pitch_mm` (直线) 或 `360.0` (旋转)。 |
| `rotation_direction` | Integer | - | 旋转方向修正：`1` (正向), `-1` (反向)。 |

### 3. 运动学参数 (Kinematics)
定义轴的运动能力上限，用于轨迹规划。

| 字段名 | 类型 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `max_velocity_units` | Float | unit/s | 允许的最大速度。 |
| `max_acceleration_units` | Float | unit/s² | 允许的最大加速度。 |
| `max_deceleration_units` | Float | unit/s² | 允许的最大减速度。 |
| `max_jerk_units` | Float | unit/s³ | 允许的最大加加速度 (Jerk)，用于 S 型曲线规划平滑度。 |
| `default_velocity_units` | Float | unit/s | 默认运行速度（当未指定速度时使用）。 |

### 4. 安全限位 (Limits)
软件层面的安全保护机制。

| 字段名 | 类型 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `soft_limit_pos_units` | Float | unit | 正向软限位位置。 |
| `soft_limit_neg_units` | Float | unit | 负向软限位位置。 |
| `max_position_error_units`| Float | unit | 允许的最大跟随误差（指令位置 - 反馈位置）。 |
| `max_current_amp` | Float | A | 允许的最大电流（用于过流保护或力矩限制参考）。 |

### 5. 回零参数 (Homing)
基于 CiA402 标准的回零配置。

| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `method` | Integer | 回零方法 (CiA402 0x6098)。<br>常用值：<br> `33`: 向负方向寻找 Index 脉冲<br> `34`: 向正方向寻找 Index 脉冲<br> `17-30`: 结合限位开关回零<br> `37`: 当前位置设为零点 |
| `speed_fast_units` | Float | 寻找限位开关/原点开关的高速速度 (unit/s)。 |
| `speed_slow_units` | Float | 寻找 Index 脉冲或精确定位的低速速度 (unit/s)。 |
| `offset_units` | Float | 回零完成后的原点偏移量 (unit)。 |
| `current_threshold_amp` | Float | 机械限位回零（硬停）时的电流阈值 (A)。 |

### 6. 同步控制 (Synchronization)
用于多轴耦合运动（如龙门双驱）。

| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `is_synced` | Boolean | 是否启用同步功能。 |
| `sync_master_axis_id` | Integer | 同步主轴的 ID。如果是主轴本身或独立轴，设为 `-1`。 |
| `sync_type` | String | 同步类型：<br> `none`: 无同步<br> `master`: 作为主轴<br> `slave`: 作为从轴 |
| `sync_direction` | Integer | 同步方向：`1` (同向), `-1` (反向)。 |

## 示例配置

```json
{
  "axis_id": 2,
  "name": "X_Axis_Slave",
  "mechanical": {
    "gear_ratio": 10.0,
    "unit_per_rev": 10.0
  },
  "synchronization": {
    "is_synced": true,
    "sync_master_axis_id": 1,
    "sync_type": "slave",
    "sync_direction": 1
  }
}
```
