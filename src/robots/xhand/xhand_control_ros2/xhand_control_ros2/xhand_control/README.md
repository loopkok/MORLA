# XHandControl SDK Usage

<p align="center">
  <a target="_blank" href="./README.zh-CN.md">中文</a>
</p>

## Introduction
The XHandControl SDK is a software development kit for communication with the XHand robotic hand, allowing control of the hand's joints and reading sensor data.

## Install Dependencies
```bash
sudo apt update && sudo apt install -y \
    cmake \
    g++ \
    libcurl4-openssl-dev \
    libssl-dev \
    nlohmann-json3-dev
```

## Build
```bash
# After downloading the project archive, please follow the steps below to extract and run it.
tar -zxf xhand_control_sdk_*.tar.gz
cd xhand_control_sdk/tests/
mkdir build/
cd build/
cmake ..
make
```

## Run
```bash
# EtherCAT Communication
sudo ./test_ethercat

# Serial Communication
# Method 1: Run directly with sudo privileges
sudo ./test_serial
# Method 2: Grant permission to the serial port (using ttyUSB0 as an example)
sudo chmod 666 /dev/ttyUSB0
./test_serial
```

## Fingure Joint Mode
- In the **FingerCommand_t** class, the **mode** parameter is used to control the mode of finger joints.
- The modes of finger joints and the meanings of the parameters are shown in the table below:

| Name   | Value   | Notes        |
|--------|:--------:|-------------|
| Passive Mode  | 0    |    |
| Position Control Mode  | 3    |    |
| Force Control Mode  | 5    |XHand hands with serial numbers prefixed with "2L3201" are not supported. Please use the parameter ```tor_max```, with its value specified in milliamperes (mA). |

## Important Statement
By using the product, the user shall be deemed to have read, understood and agreed to be bound by all provisions in the User Agreement. During the use, the user shall strictly abide by the provisions in the User Agreement and use the product reasonably and lawfully. Refer to [<a target="_blank" href="./USER_AGREEMENT.md">**USER_AGREEMENT**</a>] for the User Agreement.