# MT 电机 MIT 位置模式接入说明

> 当前状态：MIT 位置模式已完成联调，可稳定进入位置环并执行往返位置指令。

本文说明 MT 协议在本仓库中的最新行为：
- 速度模式：仍使用 `0xA2`（速度闭环）
- 位置模式：默认使用 MIT 指令（`0x400 + ID`）

> 协议依据：`docs/脉塔协议.md` 第 2.20 节（速度闭环）与第 5 章（运动模式控制指令 CAN）。

## 1. 当前实现

### 1.1 速度模式（不变）

- `setVelocity()` 发送 `0xA2`
- 速度输入按现有实现保持兼容

### 1.2 位置模式（MIT）

- `setPosition()` 默认走 MIT 帧（`0x400 + motor_id`）
- `setMode(position)` 会主动执行 MIT 预热流程（抱闸释放 + 运行模式查询 + 当前位置保持帧），降低“未完全进入运控模式”导致的不动作概率。
- 当运行模式已在位置环（`runmode=0x03`）但位置误差仍较大时，会自动追加 `A4` 辅助保持帧，提升持续跟踪稳定性（解决“首帧动、后续不动”）。
- 按协议打包 5 个参数：
  - `p_des`（期望位置，rad）
  - `v_des`（期望速度，rad/s）
  - `kp`
  - `kd`
  - `t_ff`（前馈力矩，N·m）

当前默认值：
- `kp = 80.0`
- `kd = 2.5`
- `t_ff = 0.8`

### 1.3 MIT 回包解析

支持解析 `0x500 + ID` 回包，更新：
- 位置（内部转换为 0.01° 缓存）
- 速度（转换为 dps 缓存）

同时支持解析 `0x70` 运行模式应答并打印日志：
- `[MtCan][MODE] ... runmode=0x03` 表示当前处于位置环模式（MIT/位置控制可执行）
- `runmode=0x02` 表示仍在速度环模式

新增调试日志：
- `[MtCan][MODE-PRIME] A4 hold sent ...`：表示驱动正在执行位置环入模预热。
- `[MtCan][MODE-ASSIST] A4 assist ...`：表示已在位置环下触发位置跟踪辅助。
- `TX-PLAN` 中 `primed=1 runmode=0x3`：表示当前 MIT 命令已在可执行状态下发。

## 2. 运行时开关（环境变量）

在启动 `can_driver_node` 前可配置：

- `CAN_DRIVER_MT_USE_MIT_POSITION`
  - `1/true`：位置模式使用 MIT（默认）
  - `0/false`：位置模式退回 `0xA4`

- `CAN_DRIVER_MT_MIT_DEFAULT_KP`
- `CAN_DRIVER_MT_MIT_DEFAULT_KD`
- `CAN_DRIVER_MT_MIT_DEFAULT_TORQUE`

示例：

```bash
export CAN_DRIVER_MT_USE_MIT_POSITION=1
export CAN_DRIVER_MT_MIT_DEFAULT_KP=80
export CAN_DRIVER_MT_MIT_DEFAULT_KD=2.5
export CAN_DRIVER_MT_MIT_DEFAULT_TORQUE=0.8
roslaunch can_driver can_driver.launch
```

> 若 MIT 仍不明显，可再提高到：`KP=120`、`KD=3.0`、`T_FF=1.2`（逐步调，注意机械安全）。

## 3. 多车接口（避免改代码）

### 3.1 profile 文件

- `config/mt_control_profiles.yaml`

已预置：
- `car_a`
- `car_b`

当前默认 MT ID（按 [config/can_driver.yaml](config/can_driver.yaml)）：
- `0x141`
- `0x14B`

### 3.2 接口脚本

- `scripts/mt_motor_interface.py`

示例：

```bash
# 查看 profile
rosrun can_driver mt_motor_interface.py --profile car_a --action list

# 查看当前运行时已加载的 MT joints（带 can_device 匹配标记）
rosrun can_driver mt_motor_interface.py --profile car_a --action discover

# 使能 + 速度模式 + 下发速度
rosrun can_driver mt_motor_interface.py --profile car_a --action enable   --motor-id 0x141
rosrun can_driver mt_motor_interface.py --profile car_a --action mode     --motor-id 0x141 --value 1
rosrun can_driver mt_motor_interface.py --profile car_a --action velocity --motor-id 0x141 --value 5.0

# 切 MIT 位置模式并下发位置
rosrun can_driver mt_motor_interface.py --profile car_a --action mode     --motor-id 0x141 --value 0
rosrun can_driver mt_motor_interface.py --profile car_a --action position --motor-id 0x141 --value 1.0

# 若已提前切好模式，连续下发位置时建议加 --no-auto-mode，避免每帧重复切模式
rosrun can_driver mt_motor_interface.py --profile car_a --action position --motor-id 0x141 --value 1.0 --no-auto-mode

# 停止 + 失能
rosrun can_driver mt_motor_interface.py --profile car_a --action stop     --motor-id 0x141
rosrun can_driver mt_motor_interface.py --profile car_a --action disable  --motor-id 0x141
```

## 4. 一键测试脚本

- `scripts/test_mt_motor_motion.sh`

示例：

```bash
bash scripts/test_mt_motor_motion.sh car_a 0x141 5.0 2.0 2.5 2.0
bash scripts/test_mt_motor_motion.sh car_a auto 5.0 2.0 2.5 2.0
```

说明：
- `motor_id=auto` 时，脚本会优先读取运行时 `/can_driver_node/joints` 中的 MT 电机 ID；
  若运行时不可读，则回退 `profile.mt_motor_ids`，最后再回退默认 `0x141/0x14B`。
- 脚本会在测试前自动确保生命周期进入 `Running`：
  - 若 `Faulted`：执行 `recover`
  - 若 `Inactive/Configured`：执行 `init`
  - 若 `Standby`：执行 `enable`
  - 若 `Armed`：执行 `resume`
- 若 profile 中 `can_device` 不适用现场，可通过环境变量覆盖：
  `MT_TEST_INIT_DEVICE=can0`（必要时也可设 `MT_TEST_DRIVER_NS=/can_driver_node`）
- `pos_rad` 建议先用 `2.0~3.0`，位移更明显。
- `mit_hold_sec` 是每个 MIT 位置点的停留时间（秒）。
- 脚本会在 `mit_hold_sec` 内持续流式发送 MIT 位置命令（默认 20Hz），比单次下发更容易观察到动作。
- 流式下发阶段会自动使用 `--no-auto-mode`，避免重复调用 `mode=0` 造成目标被频繁重置。
- 可通过环境变量调整流频：`MT_TEST_MIT_STREAM_HZ=30`。
- 脚本内置 `MIT-VERIFY` 强测阶段（固定 ±2.5rad 往返），用于直接验证 MIT 是否生效。

推荐验收标准：
- 日志出现并维持 `runmode=0x03`；
- `TX-PLAN` 显示 `primed=1 runmode=0x3` 且 `p_rad` 为目标值（如 ±2.5）；
- `MIT RX` 的 `p_rad` 能随目标变化，不长期停在单一点附近。

流程：
1. Enable
2. 速度模式正转
3. 速度模式反转
4. 速度归零（`velocity=0`）
5. Stop（保险）
6. 切换 MIT 位置模式
7. MIT 位置 `+pos_rad`
8. MIT 位置 `-pos_rad`
9. MIT 回零
10. Final Stop（默认）
11. MIT-VERIFY 强测（脚本内置）



如需最后失能，可加环境变量：

```bash
MT_TEST_SEND_DISABLE=1 bash scripts/test_mt_motor_motion.sh car_a 0x141 5.0 2.0 2.5 2.0
```

```bash
MT_TEST_MIT_STREAM_HZ=30 bash scripts/test_mt_motor_motion.sh car_a 0x141 5.0 2.0 2.5 2.0
```

## 5. 常见问题：脚本显示 OK 但电机不转

### 5.0 常见问题：脚本提示找不到 motor id

先执行：

```bash
rosrun can_driver mt_motor_interface.py --profile car_a --action discover
```

检查点：
- 目标 ID 是否出现在输出中；
- `joint` 对应条目的 `protocol` 是否为 `MT`；
- `can_device` 是否与当前 profile 一致（脚本/接口已支持不一致时按 `motor_id` 退化匹配，但建议最终对齐配置）。

优先检查 [config/can_driver.yaml](config/can_driver.yaml) 中 MT 关节的缩放参数是否已配置：

- `velocity_scale: 1.7453292519943296e-04`
- `position_scale: 1.7453292519943296e-04`

原因：
- MT 速度命令单位是 `0.01dps/LSB`
- MT 位置（含 MIT 输入兼容路径）按 `0.01°/LSB` 解释

如果 `scale` 误配为 `1.0`，下发 `5.0 rad/s` 会变成很小的原始值（几乎不动）。

另一个高频现象：日志中反复出现

- `p_raw_0.01deg=0`
- `p_rad=0`

这表示当前发出的 MIT 目标位置就是 0 rad（回零点），并非“没发命令”。
若你正在验证位移动作，请重点看 `+pos_rad/-pos_rad` 阶段对应的 `TX-PLAN`，确认 `p_rad` 为非零值。

如果日志显示 `runmode=0x2` 持续不变：
- 先确认已执行 `mode --value 0`；
- 再看是否出现 `[MtCan][MODE-PRIME]`；
- 若仍长时间不进入 `0x03`，优先检查驱动器状态灯和故障位（堵转/过流/欠压等）。

## 6. ECB 网络电机接入（1MINTASCA）

当前 `can_driver` 已支持在同一套硬件层中接入 ECB 网络电机，协议类型使用 `protocol: ECB`。

ECB 与 MT/PP/DM 共用标准 `config/can_driver.yaml` 和 `launch/can_driver.launch`，后端由各 joint 的 `protocol` 字段自动选择。

### 6.1 配置示例

固定 IP + ID（推荐生产环境）：

```yaml
- name: ecb_joint_fixed
  motor_id: 0x01
  protocol: ECB
  can_device: ecb://192.168.1.20
  ecb_ip: 192.168.1.20
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2
```

自动扫描（适合调试）：

```yaml
- name: ecb_joint_auto
  motor_id: 0x02
  protocol: ECB
  can_device: ecb://auto
  ecb_discovery: auto
  ecb_refresh_ms: 20
  control_mode: velocity
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2
```

### 6.2 字段说明

- `protocol: ECB`：启用 Innfos ECB 网络协议。
- `can_device`：ECB 逻辑设备名，格式 `ecb://<ip>` 或 `ecb://auto`。
- `ecb_ip`：固定 IP（可选）。
- `ecb_discovery`：`fixed` 或 `auto`，也支持布尔值（`true=auto`，`false=fixed`）。
- `ecb_refresh_ms`：后台状态刷新周期（毫秒，必须大于 0）。

### 6.3 单位约定

ECB 后端默认按以下固定点原始单位解释：

- 位置命令/反馈：`1 raw = 1e-4 rev`
- 速度命令/反馈：`1 raw = 0.1 rpm`

建议使用以下 scale 对齐到 SI 单位：

- `position_scale = 2*pi/10000 = 6.283185307179586e-4`
- `velocity_scale = (2*pi/60)/10 = 1.0471975511965978e-2`

### 6.4 接入注意事项

- 自动扫描会触发 `lookupActuators`，启动时延会高于固定 IP。
- 推荐生产环境优先固定 IP+ID，自动扫描仅用于调试或首部署。
- ECB 为网络链路，若出现抖动，优先增大 `ecb_refresh_ms` 并检查交换机/网口质量。
- 可使用 `scripts/test_ecb_motor_motion.sh` 做一键位置/速度联调。
