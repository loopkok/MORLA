#pragma once

#include <math.h>
#include <sys/mman.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ev_boot {

#define APP_ADDR_OFFSET 512  // APP运行区实际代码偏移量
#define BOOT_DATA_BUF_INDEX_LEN 2
#define BOOT_DATA_BUF_DATA_LEN 4096
#define BOOT_DATA_BUF_LEN (BOOT_DATA_BUF_INDEX_LEN + BOOT_DATA_BUF_DATA_LEN)
//升级指令
#define CMD_IAP_BEGIN 0xA1   //升级开始
#define CMD_IAP_TRANS 0xA2   //升级数据
#define CMD_IAP_VERIFY 0xA3  //升级校验
#define CMD_IAP_RESET 0xA4   // MCU复位
#define CMD_IAP_ACK 0xB0     //升级应答
//升级错误码
#define IAP_OK 0
#define IAPERROR_SIZE 1        //固件大小超范围
#define IAPERROR_ERASE 2       //擦除Flash失败
#define IAPERROR_WRITEFLASH 3  //写入Flash失败
#define IAPERROR_INDEX 5       //索引错误
#define IAPERROR_BUSY 6        //总线忙
#define IAPERROR_FORM 7        //数据格式错误
#define IAPERROR_VERIFY 8      //校验错误
#define IAPERROR_OTHER 9       //其他错误

typedef enum {
  IDLE,    //空闲
  BEGIAN,  //开始
  TRANS,   //发送数据
  VERIFY,  //校验
  RESET,   //软件重启
} firmware_update_status_t;

typedef struct {
  uint32_t retry_counter;
  uint32_t update_count;  //升级计数
  uint16_t packet_index;  //传输包索引号
  uint8_t ack;            //应答标志
  uint8_t ackResult;      //应答结果

} firmware_update_t;

class EvBoot {
 public:
  EvBoot() {}
  ~EvBoot() {}
  firmware_update_status_t firmware_update_status;
  firmware_update_t firmware_update;
  void Testsha256(void);
  void TestFirmwareUpdateSDOCom(int32_t slave);
  void FirmwareUpdate(std::string app_file_name, int32_t slave, uint8_t id);
  void AddFileInfo(std::vector<uint8_t>& buffer, uint32_t file_len,
                   uint32_t crc);
  // void Crc8x1TabCreat(void);
  // uint8_t CalculateCrc(uint8_t* crc_buf,uint32_t buf_len);
  void JumpToApp(uint32_t slave_id);
  // void UpdateBootState(McBootFile_t* boot_file, uint32_t slave_id);
  bool EnterUpdatingState(int slave_id, const char* file_name,
                          uint32_t boot_buffer_size);
  bool BootFileUpdating(uint32_t slave_id,
                        const std::vector<uint8_t>& boot_buffer);
  std::string ExtractBoardName(const std::string& original);
  int ReadBootFile(std::vector<uint8_t>& buffer, std::string app_file_name);
  int ReadBootFile(std::vector<uint8_t>& buffer, const char* board_name);
  void ResetStatus();
 private:
 private:
  // SdoCom sdo_com_;
  // std::string app_file_name_;
  // uint8_t Crc8xTable[256];
};

}  // namespace ev_boot