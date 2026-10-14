# XHandControl SDK 使用说明

<p align="center">
  <a target="_blank" href="./README.md">English</a>
</p>

## 介绍
XHandControl SDK 是用于与 XHand 机器人手进行通信的软件开发工具包，支持控制手部关节并读取传感器数据。

## 安装依赖
```bash
sudo apt update && sudo apt install -y \
    cmake \
    g++ \
    libcurl4-openssl-dev \
    libssl-dev \
    nlohmann-json3-dev
```

## 编译
```bash
# 下载本项目压缩包后，请按以下步骤解压并运行
tar -zxf xhand_control_sdk_*.tar.gz
cd xhand_control_sdk/tests/
mkdir build/
cd build/
cmake ..
make
```

## 运行
```bash
# EtherCAT 通信
sudo ./test_ethercat

# 串口通信
# 方式一：使用 sudo 权限直接启动
sudo ./test_serial
# 方式二：给串口添加权限（以 ttyUSB0 为例）
sudo chmod 666 /dev/ttyUSB0
./test_serial
```

## 手指关节模式
- 在手指类 **FingerCommand_t** 中，通过修改参数 **mode** 来控制手指关节的模式。
- 手指关节模式以及参数含义如下表所示： 

| 名称   | 数值   | 备注        |
|--------|:--------:|-------------|
| 无力模式  | 0    |    |
| 位控模式  | 3    |    |
| 力控模式  | 5    | SN（序列号）前缀包含 “2L3201” 的灵巧手设备不受支持；请使用参数 ```tor_max```，其参数值单位为电流（单位：mA）。|

## 重要声明
用户使用本产品即视为已充分阅读、理解并同意接受用户协议的全部条款约束。在使用过程中，用户应严格遵守该协议约定，合理合法地使用本产品。用户协议详见【<a target="_blank" href="./用户协议.md">**用户协议**</a>】。