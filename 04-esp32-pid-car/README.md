# ESP32-C3 BLE PID 小车

基于 ESP32-C3 的双轮小车。固件接收 BLE 运动指令，根据编码器测得的左右轮转速分别进行 PID 控制；Web Dashboard 用于下发指令和查看运行状态。

## 主要功能

- BLE 控制前进、后退、转向和停止，支持自动避障模式。
- 左右电机独立 PID 调速，并回传实测转速。
- Web Dashboard 显示转速与状态，提供操作界面。

## 我的工作

实现 BLE 指令处理、编码器测速、双电机闭环控制及网页控制界面。

## 主要技术

ESP32-C3、C / C++、BLE、编码器测速、PID、HTML / JavaScript。

## 工程目录

```text
source/
├── firmware/esp32_pid_car.ino          小车固件
└── dashboard/control_dashboard.html   Web Dashboard
```

[查看固件](source/firmware/esp32_pid_car.ino) · [查看 Web Dashboard](source/dashboard/control_dashboard.html)
