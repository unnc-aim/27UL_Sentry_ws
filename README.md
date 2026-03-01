# infantry_ws ROS2 systemd 服务操作

## 常用 systemctl 命令

### 查看状态

```bash
sudo systemctl status ros2-foxglove-bridge.service
sudo systemctl status ros2-soem-bringup.service
sudo systemctl status ros2-infantry-chassis.service
```

### 启动

```bash
sudo systemctl start ros2-foxglove-bridge.service
sudo systemctl start ros2-soem-bringup.service
sudo systemctl start ros2-infantry-chassis.service
```

### 停止

```bash
sudo systemctl stop ros2-foxglove-bridge.service
sudo systemctl stop ros2-soem-bringup.service
sudo systemctl stop ros2-infantry-chassis.service
```

### 重启

```bash
sudo systemctl restart ros2-foxglove-bridge.service
sudo systemctl restart ros2-soem-bringup.service
sudo systemctl restart ros2-infantry-chassis.service
```

### 开机自启

```bash
sudo systemctl enable ros2-foxglove-bridge.service
sudo systemctl enable ros2-soem-bringup.service
sudo systemctl enable ros2-infantry-chassis.service
```

### 取消开机自启

```bash
sudo systemctl disable ros2-foxglove-bridge.service
sudo systemctl disable ros2-soem-bringup.service
sudo systemctl disable ros2-infantry-chassis.service
```

### 查看日志

```bash
sudo journalctl -u ros2-foxglove-bridge.service -f
sudo journalctl -u ros2-soem-bringup.service -f
sudo journalctl -u ros2-infantry-chassis.service -f
```

## 批量操作

```bash
sudo systemctl start ros2-foxglove-bridge.service ros2-soem-bringup.service ros2-infantry-chassis.service
sudo systemctl stop ros2-foxglove-bridge.service ros2-soem-bringup.service ros2-infantry-chassis.service
sudo systemctl restart ros2-foxglove-bridge.service ros2-soem-bringup.service ros2-infantry-chassis.service
sudo systemctl status ros2-foxglove-bridge.service ros2-soem-bringup.service ros2-infantry-chassis.service
```

## Makefile 快捷方式

项目根目录提供了 `Makefile`，可直接执行：

```bash
make help
make status-all
make start-all
make stop-all
make restart-all
make logs SERVICE=ros2-soem-bringup.service
```
