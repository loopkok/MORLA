#ifndef _COMMUNICATION_INTERFACE_H_
#define _COMMUNICATION_INTERFACE_H_

#include <map>
#include <string>
#include <vector>

#include "data_type.hpp"
#include "error_manager.hpp"
#include "ev_boot.h"

// #define APP_ADDR_OFFSET 512  // APP运行区实际代码偏移量
// #define BOOT_DATA_BUF_INDEX_LEN 2
// #define BOOT_DATA_BUF_DATA_LEN 4096
// #define BOOT_DATA_BUF_LEN (BOOT_DATA_BUF_INDEX_LEN + BOOT_DATA_BUF_DATA_LEN)
// //升级指令
// #define CMD_IAP_BEGIN 0xA1   //升级开始
// #define CMD_IAP_TRANS 0xA2   //升级数据
// #define CMD_IAP_VERIFY 0xA3  //升级校验
// #define CMD_IAP_RESET 0xA4   // MCU复位
// #define CMD_IAP_ACK 0xB0     //升级应答
// //升级错误码
// #define IAP_OK 0
// #define IAPERROR_SIZE 1        //固件大小超范围
// #define IAPERROR_ERASE 2       //擦除Flash失败
// #define IAPERROR_WRITEFLASH 3  //写入Flash失败
// #define IAPERROR_INDEX 5       //索引错误
// #define IAPERROR_BUSY 6        //总线忙
// #define IAPERROR_FORM 7        //数据格式错误
// #define IAPERROR_VERIFY 8      //校验错误
// #define IAPERROR_OTHER 9       //其他错误

#define CMD_SET_EXPECT_PARAM 0x02       //设置指关节期望参数
#define CMD_READ_USER_PARAM 0x03        //读取指关节用户参数
#define CMD_WRITE_USER_PARAM 0x04       //写入指关节用户参数
#define CMD_BURN_FLASH 0x05             //指关节写参数到flash中
#define CMD_UPDATE_CURRENT_PARAM 0x06   //更新电流环参数
#define CMD_UPDATE_POSITION_PARAM 0x07  //更新位置环参数
#define CMD_SET_MODE 0x08               //设置模式
#define CMD_READ_MODE 0x09              //读取模式
#define CMD_QUERY_MATRIX_PRESSURE 0x11  //获取传感器数据
#define CMD_CLREAR_PRESSURE 0x12        //传感器数据清零
#define CMD_READ_SOFTWARE_VERSION 0X13  // 读取软件版本号
#define CMD_RESET 0X14                  // 软件复位重启
#define CMD_READ_REGISTER 0X15          // 读取寄存器数据
#define CMD_WRITE_REGISTER 0X16         // 写入寄存器数据
#define CMD_CALIBRATION 0X17            // 执行校准操作
#define CMD_ACTIONS_CTRL 0X18           // 动作循环播放控制
#define CMD_QUERY_ACTIONS_COUNT 0X19    // 获取动作次数计数
#define CMD_CLEAR_ACTIONS_COUNT 0X20    // 关节动作次数清零

#define CMD_ERROR 0x00  // error case

//寄存器表
#define REGISTER_TAB_SIZE 512

//寄存器地址 以下只读
#define REGISTER_WHICH_HAND 20  //哪一只手 1BYTE
#define REGISTER_SN 21          // SN码 32BYTE

//寄存器地址 以下可读可写
#define REGISTER_HAND_ID 53      // ID 1byte
#define REGISTER_HAND_NAME 54    //手名字  32字节
#define REGISTER_CALIBRATION 86  //校准参数 120字节
#define REGISTER_SILENT 206  // 静默模式 1byte

// 动作循环播放命令字
#define SUBCMD_ACTIONS 0x01
#define SUBCMD_ON_OFF 0x02

typedef enum : uint16_t {
    FS_INVALID = 0xFFFF,   // 无效状态，用于初始化等非正常状态
    FS_STATE_ERROR = 0x0000,
    FS_IDEL = 0x0001,
    FS_INIT = 0x0002,
    FS_RUNNING = 0x0003,
    FS_CONFIG = 0x0004,
} FirmwareState;

namespace communication_interface {
class CommunicationInterface {
 public:
  virtual std::vector<std::string> enumerate_devices() {
    // C++代码实现
    return {};
  }
  virtual bool open_device(xhand_control::ErrorStructPtr error) = 0;
  virtual std::vector<uint8_t> list_hands_id() = 0;
  virtual DeviceInfo_t read_device_info(
      uint8_t device_id, xhand_control::ErrorStructPtr error) = 0;

  virtual bool send_command(const uint8_t device_id,
                            const HandCommand_t& command,
                            xhand_control::ErrorStructPtr error) = 0;
  virtual HandState_t read_state(uint8_t device_id,
                                 xhand_control::ErrorStructPtr error,
                                 bool force_update = false) = 0;

  virtual HandParam_t read_joint_parameters(
      uint8_t device_id, xhand_control::ErrorStructPtr error) = 0;

  virtual bool set_joint_parameters(const uint8_t device_id,
                                    const HandParam_t& parameters,
                                    xhand_control::ErrorStructPtr error) = 0;

  virtual bool upgrade_device(const uint8_t device_id,
                              const uint8_t component_id,
                              const std::string& firmware_path,
                              xhand_control::ErrorStructPtr error) {
    // C++代码实现
    return true;
  }

  // virtual bool calibrate_joint(uint8_t device_id) {
  //   // C++代码实现
  //   return true;
  // }

  virtual std::string get_firmware_version(
      const uint8_t device_id, const uint8_t component_id,
      xhand_control::ErrorStructPtr error) = 0;

  virtual bool reset_sensor(const uint8_t device_id, const uint8_t sensor_id,
                            xhand_control::ErrorStructPtr error) = 0;

  virtual void close_device() {}

  virtual char get_hand_type(uint8_t device_id,
                             xhand_control::ErrorStructPtr error) = 0;
  virtual std::string get_serial_number(
      uint8_t device_id, xhand_control::ErrorStructPtr error) = 0;
  virtual bool set_hand_name(const uint8_t device_id, const std::string& name,
                             xhand_control::ErrorStructPtr error) = 0;
  virtual std::string get_hand_name(uint8_t device_id,
                                    xhand_control::ErrorStructPtr error) = 0;
  virtual bool set_hand_id(uint8_t old_id, uint8_t new_id,
                           xhand_control::ErrorStructPtr error) = 0;
  // virtual uint8_t get_hand_id(uint8_t device_id) = 0;
  virtual bool set_hand_parameters(const uint8_t device_id,
                                   const HandParam_t& parameters,
                                   xhand_control::ErrorStructPtr error) = 0;
  virtual HandParam_t get_hand_parameters(
      uint8_t device_id, xhand_control::ErrorStructPtr error) = 0;
  virtual bool action_group_set(const uint8_t device_id,
                        const HandActionGroup_t& hand_angle_pose,
                        xhand_control::ErrorStructPtr error) = 0;
  virtual bool action_group_run(const uint8_t device_id,
                        const HandActioCtrl_t& hand_action_ctrl,
                        xhand_control::ErrorStructPtr error) = 0;
  virtual uint16_t read_firmware_state(const uint8_t device_id,
                        const uint8_t component_id,
                        xhand_control::ErrorStructPtr error) = 0;
  virtual bool set_firmware_state(const uint8_t device_id,
                        const uint8_t component_id,
                        const uint16_t state,
                        xhand_control::ErrorStructPtr error,
                        uint32_t timeout_ms) = 0;  
  virtual bool set_silent_mode(const uint8_t device_id, uint8_t mode, xhand_control::ErrorStructPtr error) = 0;
  virtual int8_t read_silent_mode(const uint8_t device_id, xhand_control::ErrorStructPtr error) = 0;
  virtual std::vector<std::vector<uint64_t>> get_action_counts(const uint8_t device_id, 
                                                xhand_control::ErrorStructPtr error,
                                                uint32_t timeout_ms) = 0;                                    
  virtual bool clear_actions_count(const uint8_t device_id, 
                                                  xhand_control::ErrorStructPtr error,
                                                  uint32_t timeout_ms) = 0;

 protected:
  std::shared_ptr<ev_boot::EvBoot> ev_boot_;

 private:
};
}  // namespace communication_interface

#endif