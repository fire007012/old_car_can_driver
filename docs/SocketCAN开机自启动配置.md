# SocketCAN 开机自启动配置

本项目使用 `can0` 和 `can1` 分别承载不同电机链路。两张 CAN 卡不能因为插入顺序变化而交换编号，因此开机配置脚本采用“安装时当前系统里的 `can0`/`can1` 就是基准”的策略：记录当前两张卡的 USB 序列号或路径，后续开机时先把对应硬件恢复成记录的接口名，再配置 CAN 参数并拉起接口。

## 安装

确认两张 CAN 卡已经插好，并且当前名字就是期望的最终名字：

```bash
ip -details link show type can
```

安装自启动服务：

```bash
cd ~/catkin_ws/src/robot24/can_driver
sudo scripts/socketcan_boot_setup.sh install
```

安装会记录当前映射到 `/etc/robot24/socketcan_boot.conf`，并启用 `robot24-socketcan.service`。默认参数：

- `bitrate=1000000`
- `txqueuelen=1000`
- 处理接口：`can0`、`can1`

## 立即应用或查看状态

```bash
sudo scripts/socketcan_boot_setup.sh apply
scripts/socketcan_boot_setup.sh status
```

`apply` 会执行以下动作：

1. 按安装时记录的硬件身份恢复 `can0` 和 `can1` 名称。
2. 对存在的接口执行 `ip link set dev <iface> down`。
3. 设置 `type can bitrate 1000000`。
4. 设置 `txqueuelen 1000`。
5. 拉起接口。

## 重新绑定或卸载

如果确实要改变哪张硬件卡对应 `can0` 或 `can1`，先手动调整到期望状态，然后重新运行：

```bash
sudo scripts/socketcan_boot_setup.sh install
```

卸载自启动服务和记录文件：

```bash
sudo scripts/socketcan_boot_setup.sh uninstall
```