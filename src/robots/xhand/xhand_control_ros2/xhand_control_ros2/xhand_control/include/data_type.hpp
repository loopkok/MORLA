#ifndef _DATA_TYPE_HPP_
#define _DATA_TYPE_HPP_

#include <array>
#include <cstdint>
#include <cstring>

#define EV_HAND_JOINT_NUM 12

typedef struct {
  std::array<int16_t, 12> position_stretching;
  std::array<int16_t, 12> position_closing;
  std::array<int16_t, 12> position_zero;
  std::array<int16_t, 12> position_init;
  std::array<int8_t, 12> angle_stretching;
  std::array<int8_t, 12> angle_closing;
} __attribute__((packed)) HandParam_t;

// 新校准参数
typedef struct {
  std::array<char, 11> tools; // [S1, D1]
  uint8_t step;   // [1,2]
  std::array<int8_t, 12> calibrated_angles;        // 标定角度（单位：度） ： 从spec获取
  std::array<int8_t, 12> lower_angles;             // 上位机获取的下限角度 ： 从spec获取
  std::array<int8_t, 12> upper_angles;             // 上位机获取的上限角度 ： 从spec获取
} __attribute__((packed)) CalibrationParams;

typedef union {
  HandParam_t handparam;                  
  CalibrationParams calibr_params;
} CalibrationUnion;

typedef struct {
  uint16_t resverse;
  char ev_hand{'N'};  //左手 ‘L’/‘l’ 还是右手 ‘R’/‘r’
  uint8_t hand_id;    //灵巧手ID
  std::array<char, 32> serial_number;  //序列号
  std::array<char, 32> name;           //灵巧手名称
  HandParam_t hand_param;              //校准参数
  uint8_t iap_flag;                    //升级标志
  uint8_t is_calibrated;
} __attribute__((packed)) DeviceInfo_t;


typedef enum : uint8_t {
  CALIBRATION_SUCCESS = 0,                     // 操作成功
  CALIBRATION_ERROR_ANGLE_OUT_OF_RANGE,        // 角度超出范围
  CALIBRATION_ERROR_ADC_OUT_OF_RANGE,          // ADC 超出范围
  CALIBRATION_ERROR_ADC_PRE_UPPER_ERROR,       // ADC 预测上边界参数错误
  CALIBRATION_ERROR_ADC_PRE_LOWER_ERROR,       // ADC 预测下边界参数错误
  CALIBRATION_SAVE_PARAM_ERROR,                // 校准时保存参数错误
  CALIBRATION_INPUT_PARAM_ERROR,               // 校准输入参数错误
} CalibrationStatus;

typedef struct {
  CalibrationStatus calib_status; // 校准状态信息, uint8_t
  uint16_t joint_number;          // 错误时返回错误的关节number（0~11）或者0xffff（不对应某一关节参数时）
  // 只有CALIBRATION_ERROR_ADC_OUT_OF_RANGE / CALIBRATION_ERROR_ADC_PRE_UPPER_ERROR / CALIBRATION_ERROR_ADC_PRE_LOWER_ERROR这三个错误会携带参数，其它错误时参数默认为0.
  uint16_t calibrated_angle;    // 错误值：校准角度
  uint16_t calibrated_adc_value;// 错误值：校准原始值
  uint16_t intercept;           // 错误值：校准插值
} __attribute__((packed)) CalibrationErrorInfo;

typedef struct {
  int8_t fx;
  int8_t fy;
  uint8_t fz;
} __attribute__((packed)) PXSR_ForceData;

typedef struct {
  PXSR_ForceData calc_force;
  std::array<PXSR_ForceData, 120> raw_force;
  std::array<uint8_t, 20> temperature;
  uint8_t calc_temperature;
} __attribute__((packed)) SenserData_t;

typedef struct {
  uint64_t id : 8; /* Subindex1 - cur_id  5bit*/
  uint64_t sensor_id : 8;
  int64_t position : 32;      /* Subindex2 - current_pos 12bit*/
  uint64_t torque : 16;       /* Subindex3 - cur_torq 9bit*/
  uint64_t raw_position : 16; /* Subindex4 - res1 */
  uint64_t temperature : 16;
  uint64_t commboard_err : 16;  /* Subindex5 - res2 */
  uint64_t jonitboard_err : 16; /* Subindex6 - res3 */
  uint64_t tipboard_err : 16;   /* Subindex7 - res4 */
  uint64_t default5 : 16;
  uint64_t default6 : 16;
  uint64_t default7 : 16;
} __attribute__((packed)) RawFingerState_t;

typedef struct {
  RawFingerState_t raw_finger_state;  //
  SenserData_t sensor_data;  //一个指尖传感器数据，uint64_t[48]
} __attribute__((packed)) UpState_t;  //上报数据结构体

typedef struct {
  std::array<RawFingerState_t, 12> finger_state;
  std::array<SenserData_t, 5> sensor_data;
} __attribute__((packed)) SerialHandState_t;

typedef struct {
  uint8_t id;
  uint8_t sensor_id;
  float position;
  int16_t torque;   // 返回的实时电流mA 方向：[+ 手指运动背离掌心张开即⻆度趋于0°，- 手指方向朝向掌心方向运动（闭合）⻆度趋于最大⻆度]
  uint16_t raw_position;
  uint16_t temperature;
  uint16_t commboard_err;
  uint16_t jonitboard_err;
  uint16_t tipboard_err;
  int16_t default5;  // 1000倍关节⻆速度 (弧度) 实际⻆速度为:(default5/1000.0f)
  uint16_t default6;
  uint16_t default7;
} __attribute__((packed)) FingerState_t;

typedef struct {
  std::array<FingerState_t, 12> finger_state;
  std::array<SenserData_t, 5> sensor_data;
} __attribute__((packed)) HandState_t;

typedef enum : uint16_t {
  PASSIVE_MODE = 0,              // 无力模式
  POSITION_CONTROL_MODE = 3,     // 位置模式
  FORCE_CONTROL_MODE = 5,        // 力控模式
} FingerMode;

typedef struct {
  uint16_t id;
  int16_t kp;        // 比例增益参数
  int16_t ki;        // 积分系数
  int16_t kd;        // 微分增益参数
  float position;    // 设定位置
  int16_t tor_max;  // 力矩上限
  uint16_t mode;     // 模式
  uint16_t res0;     // force x (L8), force y (H8)
  uint16_t res1;     // force z (L8),
  uint16_t res2;
  uint16_t res3;
} __attribute__((packed)) FingerCommand_t;

typedef struct {
  std::array<FingerCommand_t, 12> finger_command;
} __attribute__((packed)) HandCommand_t;

typedef struct {
  std::array<int8_t, EV_HAND_JOINT_NUM> poses;  // 关节角度范围：-90° ~ 110°​
  uint16_t msec;                    // 插值时间，单位：毫秒 (ms)  默认 500​
} __attribute__((packed)) HandAnglePose_t;

// 动作序列
typedef struct {
  uint16_t action_num;                    // 动作帧的数量  2，24
  std::array<HandAnglePose_t, 24> actions;            // 动态数组，长度为 action_num
} __attribute__((packed)) HandActionGroup_t;

// 执行指令
typedef struct {
  uint16_t on_off;                    // 动作序列 启1 停0控制
  uint16_t time;                      // 动作之间插值时间：默认0，最小 50  gloable time
  uint16_t count;                     // 0 循环， > 0 实际循环次数
} __attribute__((packed)) HandActioCtrl_t;

// 动作序列反馈错误信息
typedef struct {
  uint8_t frame_id;   // 出错的帧序号（从1开始）​，0 表示没有出错
  uint8_t joint_id;   // 如果是关节角度错误，标记出错的关节序号（0~11）​，255  表示没有出错
  uint8_t time;       // 如果是插值时间错误，该值置 1​
  uint8_t flash_err;  // 保存动作组出错
} __attribute__((packed)) HandActionErrorInfo_t;

// 动作序列反馈
typedef struct {
  uint16_t status;  // 状态码：1 表示成功，0 表示失败​
  HandActionErrorInfo_t params;
} __attribute__((packed)) HandActionFeedback_t;

// 动作次数计数器
typedef struct
{
  uint64_t joint_id: 4;
  uint64_t action_counter: 30;
  uint64_t direction_change_counter: 30;

  // --------------- Getter 函数 ---------------
  uint64_t get_joint_id() const {
      uint64_t buffer;
      // 安全复制内存（避免直接指针转换）
      memcpy(&buffer, this, sizeof(buffer));
      return (buffer >> 0) & 0x0F;  // 提取低4位
  }

  uint64_t get_action_counter() const {
      uint64_t buffer;
      memcpy(&buffer, this, sizeof(buffer));
      return (buffer >> 4) & 0x3FFFFFFF;  // 提取4-33位（30 bits）
  }

  uint64_t get_direction_change_counter() const {
      uint64_t buffer;
      memcpy(&buffer, this, sizeof(buffer));
      return (buffer >> 34) & 0x3FFFFFFF;  // 提取34-63位（30 bits）
  }

  // --------------- Setter 函数 ---------------
  void set_joint_id(uint64_t value) {
      uint64_t buffer;
      memcpy(&buffer, this, sizeof(buffer));
      // 清空旧值并设置新值
      buffer = (buffer & ~0x0FULL) | ((value & 0x0F) << 0);
      memcpy(this, &buffer, sizeof(buffer));
  }

  void set_action_counter(uint64_t value) {
      uint64_t buffer;
      memcpy(&buffer, this, sizeof(buffer));
      buffer = (buffer & ~(0x3FFFFFFFULL << 4)) | ((value & 0x3FFFFFFF) << 4);
      memcpy(this, &buffer, sizeof(buffer));
  }

  void set_direction_change_counter(uint64_t value) {
      uint64_t buffer;
      memcpy(&buffer, this, sizeof(buffer));
      buffer = (buffer & ~(0x3FFFFFFFULL << 34)) | ((value & 0x3FFFFFFF) << 34);
      memcpy(this, &buffer, sizeof(buffer));
  }

} __attribute__((packed)) JointActionCounter_t;

typedef struct {
  std::array<JointActionCounter_t, 12> hand_action_counter;
} __attribute__((packed)) HandActionCounter_t;

#endif  // _DATA_TYPE_HPP_