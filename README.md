# can_driver

`can_driver` 是基于 SocketCAN 的 ROS1 电机驱动包，负责把私有 CAN 协议设备接入 `ros_control`，并提供生命周期服务、直连调试接口、协议适配和测试工具。当前同时覆盖 `MT`、`PP` 和 `DM` 后端；其中当前整车前轮/底盘主电机执行基线已切到 `MT`，`DM` 仅保留为兼容后端能力与历史接入文档参考。

## 包结构

```text
can_driver/
|-- action/
|   |-- HomeMotor.action
|   `-- MoveMotor.action
|-- config/
|   |-- can_driver.yaml
|   |-- mt_control_profiles.yaml
|   `-- ros_controllers.yaml
|-- docs/
|   |-- README.md
|   `-- 归档/
|-- launch/
|   `-- can_driver.launch
|-- msg/
|   `-- MotorState.msg
|-- scripts/
|   |-- realtime_motor_ui.py
|   |-- mt_motor_interface.py
|   `-- test_*.sh
|-- src/
|   |-- can_driver_node.cpp
|   |-- CanDriverHW.cpp
|   |-- CanDriverRuntime.cpp
|   |-- driver_ros_endpoints.cpp
|   |-- lifecycle_service_gateway.cpp
|   `-- *_protocol.cpp
|-- srv/
|   |-- Init.srv
|   |-- Recover.srv
|   |-- Shutdown.srv
|   |-- MotorCommand.srv
|   `-- SetZero*.srv / ApplyLimits.srv
`-- tests/
    `-- test_*.cpp
```

## 快速开始

编译：

```bash
cd ~/robot24_ws
catkin_make --pkg can_driver
source devel/setup.bash
```

默认启动是“驱动节点已起，但不自动加载 ros_control 控制器”：

```bash
roslaunch can_driver can_driver.launch
```

如果要一并加载控制器：

```bash
roslaunch can_driver can_driver.launch \
  use_controllers:=true \
  start_controllers:="joint_state_controller arm_controller wheel_controller"
```

## 常用命令

查看生命周期和状态：

```bash
rosservice list | grep can_driver_node
rostopic echo /can_driver_node/motor_states
rostopic echo /can_driver_node/lifecycle_state
```

标准生命周期：

```bash
rosservice call /can_driver_node/init "{}"
rosservice call /can_driver_node/enable "{}"
rosservice call /can_driver_node/disable "{}"
rosservice call /can_driver_node/halt "{}"
rosservice call /can_driver_node/resume "{}"
rosservice call /can_driver_node/recover "{}"
rosservice call /can_driver_node/shutdown "{}"
```

直连单电机调试：

```bash
rostopic pub -1 /can_driver_node/motor/left_wheel/cmd_velocity std_msgs/Float64 'data: 30.0'
rostopic pub -1 /can_driver_node/motor/arm_joint_1/cmd_position std_msgs/Float64 'data: 0.2'
```

维护服务：

```bash
rosservice call /can_driver_node/motor_command "{motor_id: 1, command: 3, value: 2}"
rosservice call /can_driver_node/set_zero "{motor_id: 1}"
rosservice call /can_driver_node/set_zero_limit "{motor_id: 1}"
```

## 接口速查

节点：

- `can_driver_node`
  - 入口：`src/can_driver_node.cpp`
  - 主体：`CanDriverHW + controller_manager + runtime/services/endpoints`

默认订阅：

- `~motor/<joint>/cmd_velocity` `std_msgs/Float64`
- `~motor/<joint>/cmd_position` `std_msgs/Float64`

默认发布：

- `~motor_states` `can_driver/MotorState`
- `~lifecycle_state` `std_msgs/String`

默认服务：

- `~init`
- `~enable`
- `~disable`
- `~halt`
- `~resume`
- `~recover`
- `~shutdown`
- `~motor_command`
- `~set_zero`
- `~apply_limits`
- `~set_zero_limit`

关键消息 / 服务：

- `msg/MotorState.msg`
- `srv/MotorCommand.srv`
- `srv/Init.srv`
- `srv/Recover.srv`
- `srv/Shutdown.srv`

关键参数：

- `config/can_driver.yaml`
  - 总线设备、关节映射、协议类型、驱动频率、超时和限位策略
- `config/ros_controllers.yaml`
  - `ros_control` 控制器定义
- `launch/can_driver.launch`
  - 是否加载控制器、启动哪些控制器

## 使用边界

- 默认 launch 不自动启动控制器，便于先做底层连通性调试。
- `direct motor` 话题适合单电机排障，不应替代正式 `ros_control` 控制链。
- `MoveMotor.action` / `HomeMotor.action` 目前只保留为消息合同，不是默认 launch 的主要操作入口。
- 若要接入统一硬件门面，优先看 `Eyou_ROS1_Master`，不要在上层重复实现生命周期编排。

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 当前文档导航与归档入口
- 推荐先读：
  - `docs/配置文件字段详解与从零配置指南.md`
  - `docs/使用指南.md`
  - `docs/架构设计.md`
  - `docs/MT_MIT模式接入说明.md`
  - `docs/达妙履带接入说明.md`（历史 DM 接入说明，非当前前轮执行基线）
