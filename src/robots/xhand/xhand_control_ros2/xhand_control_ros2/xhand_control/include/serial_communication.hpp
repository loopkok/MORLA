#ifndef _SERIAL_COMMUNICATION_HPP_
#define _SERIAL_COMMUNICATION_HPP_
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>

#include "communication_interface.hpp"
#include "parse_data.h"
#include "serial/serial.h"

namespace serial_communication {
class SerialCommunication
    : public communication_interface::CommunicationInterface {
 public:
  SerialCommunication(const std::string& port_name, int baud_rate);
  SerialCommunication() = default;
  ~SerialCommunication();
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
                        const HandActionGroup_t & hand_angle_pose,
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
                        uint32_t timeout_ms = 1000);
  virtual bool set_silent_mode(const uint8_t device_id, uint8_t mode, xhand_control::ErrorStructPtr error);
  virtual int8_t read_silent_mode(const uint8_t device_id, xhand_control::ErrorStructPtr error);
  
  virtual std::vector<std::vector<uint64_t>> get_action_counts(const uint8_t device_id, 
                                                xhand_control::ErrorStructPtr error,
                                                uint32_t timeout_ms = 1000);
  virtual bool clear_actions_count(const uint8_t device_id, 
                    xhand_control::ErrorStructPtr error,
                    uint32_t timeout_ms = 1000);

 private:
  void parse_callback();
  bool package_and_write(uint8_t target, uint8_t cmd,
                         const std::vector<uint8_t>& data,
                         xhand_control::ErrorStructPtr error,
                         uint16_t mask = 0x80, uint32_t timeout_ms = 100);
  bool package_and_write(uint8_t target, uint8_t cmd, const uint8_t* data,
                         const uint32_t len,
                         xhand_control::ErrorStructPtr error,
                         uint16_t mask = 0x80, uint32_t timeout_ms = 100);
  std::vector<uint8_t> package(uint8_t target, uint8_t cmd, const uint8_t* data,
                               uint16_t len, uint16_t mask = 0x80);
  bool read_register(const uint8_t device_id, uint16_t index, uint16_t len,
                     xhand_control::ErrorStructPtr error,
                     const int16_t slave_id = -1);
  bool write_register(const uint8_t device_id, uint16_t index, uint8_t* data,
                      uint16_t len, xhand_control::ErrorStructPtr error);
  bool write_to_device(uint8_t device_id, xhand_control::ErrorStructPtr error);
  bool write_and_wait_for_response(uint16_t device_id, const uint8_t* data,
                                   uint16_t len,
                                   xhand_control::ErrorStructPtr error,
                                   uint32_t timeout_ms = 100);
  void update_hand_state();
  int changCalibrationErrorCode(const uint8_t& error_code);

 private:
  std::shared_ptr<serial::Serial> serial_;
  std::mutex mutex_;
  std::shared_ptr<std::thread> read_thread_;
  DeviceInfo_t device_info_;
  Parse_Struct_t parse_usb_cdc_;
  SerialHandState_t up_state_;
  HandState_t hand_state_;
  std::atomic<bool> exit_{false};
  std::atomic<int> receive_flag_{0};
  std::string joint_version_;
  uint8_t hand_register_tab_[512]{0};
  xhand_control::ErrorStruct error_;
  HandCommand_t hand_command_;
  std::atomic<bool> state_{false};
  uint16_t current_firmware_state_;
  HandActionCounter_t hand_action_counter_;

  bool judge_device_id(uint8_t device_id, xhand_control::ErrorStructPtr error);
};
}  // namespace serial_communication

#endif  // _SERIAL_COMMUNICATION_HPP_