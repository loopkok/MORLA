#ifndef _ETHERCAT_COMMUNICATION_HPP_
#define _ETHERCAT_COMMUNICATION_HPP_
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>

#include "communication_interface.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"

#ifdef __cplusplus
}
#endif
#include "parse_data.h"

namespace ethercat_communication {
using PdModePdoOutputType = FingerCommand_t;

using PdModePdoInputType = UpState_t;

class EthercatCommunication
    : public communication_interface::CommunicationInterface {
 public:
  EthercatCommunication(const std::string& ifname);
  EthercatCommunication() = default;
  ~EthercatCommunication();
  virtual std::vector<std::string> enumerate_devices() override;
  bool open_device(xhand_control::ErrorStructPtr error);
  virtual std::vector<uint8_t> list_hands_id();

  virtual DeviceInfo_t read_device_info(
      uint8_t device_id, xhand_control::ErrorStructPtr error) override;

  virtual bool send_command(const uint8_t device_id,
                            const HandCommand_t& command,
                            xhand_control::ErrorStructPtr error) override;

  virtual HandState_t read_state(uint8_t device_id,
                                 xhand_control::ErrorStructPtr error,
                                 bool force_update = false) override;

  virtual HandParam_t read_joint_parameters(
      uint8_t device_id, xhand_control::ErrorStructPtr error) override;

  virtual bool set_joint_parameters(
      const uint8_t device_id, const HandParam_t& parameters,
      xhand_control::ErrorStructPtr error) override;

  virtual bool upgrade_device(const uint8_t device_id,
                              const uint8_t component_id,
                              const std::string& firmware_path,
                              xhand_control::ErrorStructPtr error) override;

  // virtual bool calibrate_joint(uint8_t device_id) override;

  virtual bool reset_sensor(const uint8_t device_id, const uint8_t sensor_id,
                            xhand_control::ErrorStructPtr error) override;
  virtual void close_device() override;
  virtual std::string get_firmware_version(
      const uint8_t device_id, const uint8_t component_id,
      xhand_control::ErrorStructPtr error) override;
  virtual char get_hand_type(uint8_t device_id,
                             xhand_control::ErrorStructPtr error) override;
  virtual std::string get_serial_number(
      uint8_t device_id, xhand_control::ErrorStructPtr error) override;
  virtual bool set_hand_name(const uint8_t device_id, const std::string& name,
                             xhand_control::ErrorStructPtr error) override;
  virtual std::string get_hand_name(
      uint8_t device_id, xhand_control::ErrorStructPtr error) override;
  virtual bool set_hand_id(uint8_t old_id, uint8_t new_id,
                           xhand_control::ErrorStructPtr error) override;
  // virtual uint8_t get_hand_id(uint8_t device_id) override;
  virtual bool set_hand_parameters(
      const uint8_t device_id, const HandParam_t& parameters,
      xhand_control::ErrorStructPtr error) override;
  virtual HandParam_t get_hand_parameters(
      uint8_t device_id, xhand_control::ErrorStructPtr error) override;
  virtual bool action_group_set(const uint8_t device_id,
                        const HandActionGroup_t& hand_angle_pose,
                        xhand_control::ErrorStructPtr error) override;
  virtual bool action_group_run(const uint8_t device_id,
                        const HandActioCtrl_t& hand_action_ctrl,
                        xhand_control::ErrorStructPtr error) override;
  virtual uint16_t read_firmware_state(const uint8_t device_id,
                        const uint8_t component_id,
                        xhand_control::ErrorStructPtr error);
  virtual bool set_firmware_state(const uint8_t device_id,
                        const uint8_t component_id,
                        uint16_t state,
                        xhand_control::ErrorStructPtr error,
                        uint32_t timeout_ms=1000);
  virtual bool set_silent_mode(const uint8_t device_id, uint8_t mode, xhand_control::ErrorStructPtr error);
  virtual int8_t read_silent_mode(const uint8_t device_id, xhand_control::ErrorStructPtr error);
  virtual std::vector<std::vector<uint64_t>> get_action_counts(const uint8_t device_id, 
                                                xhand_control::ErrorStructPtr error,
                                                uint32_t timeout_ms = 1000);
  virtual bool clear_actions_count(const uint8_t device_id, 
                        xhand_control::ErrorStructPtr error,
                        uint32_t timeout_ms = 1000);

 private:
  int set_watch_dog(ec_slavet* slave, int milliseconds);
  void ethercat_update();
  void add_timespec(struct timespec* ts, int64 addtime);
  void cycle_delay(struct timespec ts);
  static int servo_setup(uint16 slave);
  std::vector<uint8_t> package(uint8_t target, uint8_t cmd, const uint8_t* data,
                               uint16_t len, uint16_t mask = 0x80);
  void parse_callback();
  bool write_to_device(uint8_t device_id, xhand_control::ErrorStructPtr error);
  std::vector<uint8_t> send_sdo(uint16_t slave_id, const uint8_t* data,
                                uint16_t len, uint32_t timeout_ms = 100);
  void init_map(xhand_control::ErrorStructPtr error);
  DeviceInfo_t scan_hand_info(uint8_t slave_id, uint8_t device_id,
                              xhand_control::ErrorStructPtr error);
  bool read_register(const uint8_t device_id, uint16_t index, uint16_t len,
                     xhand_control::ErrorStructPtr error,
                     const int16_t slave_id = -1);
  bool write_register(const uint8_t device_id, uint16_t index, uint8_t* data,
                      uint16_t len, xhand_control::ErrorStructPtr error);
  void stop_update_thread();
  int16_t get_hand_id(uint8_t slave_id, xhand_control::ErrorStructPtr error);

  int changCalibrationErrorCode(const uint8_t& error_code);

 private:
  std::mutex mutex_;
  std::shared_ptr<std::thread> ethercat_update_thread_;
  DeviceInfo_t device_info_;
  // key is slave ids
  std::map<uint8_t, HandState_t> hand_state_;
  // key is slave ids
  std::map<uint8_t, HandCommand_t> hand_command_;
  std::atomic<bool> exit_{false};
  std::string ifname_;
  char IOmap[4096];
  std::atomic<bool> sdo_update_{false};
  // PdModePdoOutputType* pdo_output_;
  // PdModePdoInputType* pdo_input_;
  int32_t expected_wkc_{0};
  std::atomic<int32_t> actual_wkc_{0};
  int finger_index_{0};
  Parse_Struct_t parse_usb_cdc_;
  std::string joint_version_;
  std::map<uint8_t, uint8_t> device_id_to_slave_map_;
  uint8_t hand_register_tab_[512]{0};
  xhand_control::ErrorStruct sdo_error_;
//   xhand_control::ErrorStruct pdo_error_;
  uint16_t current_firmware_state_;
  HandActionCounter_t hand_action_counter_;
  
  bool judge_device_id(uint8_t device_id, xhand_control::ErrorStructPtr error);
};
}  // namespace ethercat_communication

#endif  // _ETHERCAT_COMMUNICATION_HPP_