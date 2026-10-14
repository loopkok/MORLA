#ifndef __PARSE_DATA_H__
#define __PARSE_DATA_H__

// #ifdef __cplusplus

// extern "C" {

// #endif

#include <stdbool.h>

#include <functional>

#include "stdint.h"

// #define POWER_BOARD_ID 0x01  //电源板ID
// #define HOST_ID 0xFE         //上位机ID

#define FRAME_DATA_LEN_MAX 8192 + 32  //帧结构中数据段最大长度

// typedef void (*rec_frame_callback)(void *arg);
using rec_frame_cb_t = std::function<void()>;

/**
 *  数据接收状态
 */
typedef enum {
  GET_IDEL,
  GET_HEAD,        //接收到帧头
  GET_SOURCE_ID,   //获取源ID
  GET_TARGET_ID,   //获取目的ID
  GET_CMD,         //获取命令字
  GET_DATA_LEN_L,  //获取到数据段长度低字节
  GET_DATA_LEN_H,  //获取到数据段长度高字节
  GET_FRAME,       //获取到数据帧
  GET_CRC,         //获取CRC
  GET_COMPLY       //接收完整
} Get_Data_Sta_t;

/**
 * 数据通信协议结构体
 */

typedef struct {
  uint8_t head1;                           //帧头1
  uint8_t head2;                           //帧头2
  uint8_t SourceID;                        //源ID
  uint8_t TargetID;                        //目标ID
  uint8_t Cmd;                             //命令字
  uint8_t FrameDataLen_L;                  //数据段长度低
  uint8_t FrameDataLen_H;                  //数据段长度高
  uint8_t frame_data[FRAME_DATA_LEN_MAX];  //数据段
  uint16_t crc;                            //校验和
} __attribute__((packed)) Frame_Struct_t;

typedef struct {
  Get_Data_Sta_t sta;
  Frame_Struct_t Frame_Struct;
  uint16_t get_data_len;
  rec_frame_cb_t rec_frame_cb;

} Parse_Struct_t;

void parse_struct_init(Parse_Struct_t *parse_s);
void parse_set_rec_callback(Parse_Struct_t *parse_s,
                            rec_frame_cb_t rec_frame_cb);
bool check_frame(Parse_Struct_t *parse_s);
void parse_data(Parse_Struct_t *parse_s, uint8_t *receive_buffer,
                uint16_t receive_len);

uint16_t protocolCreatPacket(uint8_t *tx_buf, uint8_t source_id,
                             uint8_t target_id, uint8_t cmd,
                             const uint8_t *data, uint16_t length);
// #ifdef __cplusplus
// }

// #endif

#endif
