// Test-only LD_PRELOAD shim. No serial device is opened and no target is sent.
#include "xhand_control.hpp"
#include <cstdlib>
#include <fstream>
#include <unistd.h>

namespace xhand_control {
static void mark(const char *name) {
  std::ofstream(std::string(std::getenv("PIANO_XHAND_TEST_DIR")) + "/" + name) << "1";
}
ErrorStruct XHandControl::open_serial(const std::string&, uint32_t) {
  mark("open_stub");
  return std::getenv("PIANO_XHAND_FAIL_INIT") ? ErrorStruct(1,"injected init failure") : ErrorStruct();
}
std::vector<uint8_t> XHandControl::list_hands_id() { return {0}; }
std::pair<ErrorStruct,std::string> XHandControl::get_hand_name(uint8_t) { return {{},"test"}; }
std::pair<ErrorStruct,std::string> XHandControl::get_hand_type(uint8_t) { return {{},"test"}; }
std::pair<ErrorStruct,HandState_t> XHandControl::read_state(uint8_t, bool force) {
  mark(force ? "UNEXPECTED_FORCE_UPDATE" : "cached_read");
  HandState_t state{};
  for (size_t i=0;i<state.finger_state.size();++i) state.finger_state[i].position=.1f+.01f*i;
  if (std::getenv("PIANO_XHAND_DIAGNOSTIC_SAMPLE")) {
    auto &j = state.finger_state[4];
    j.raw_position=1234; j.temperature=31; j.torque=-12;
    j.commboard_err=1; j.jonitboard_err=2; j.tipboard_err=4;
  }
  usleep(2000);
  if (std::ifstream(std::string(std::getenv("PIANO_XHAND_TEST_DIR"))+"/fail_read"))
    return {ErrorStruct(1,"injected device timeout"),state};
  return {{},state};
}
ErrorStruct XHandControl::send_command(uint8_t,const HandCommand_t& command) {
  const bool active = command.finger_command[0].mode == 3;
  if (active && !std::ifstream(std::string(std::getenv("PIANO_XHAND_TEST_DIR"))+"/allow_active")) mark("UNEXPECTED_COMMAND");
  for (size_t i=0;i<command.finger_command.size();++i) {
    const auto& j=command.finger_command[i];
    if (j.id!=i || j.res0!=0 || j.res1!=0 || j.res2!=0 || j.res3!=0) mark("UNEXPECTED_COMMAND");
    if (!active && (j.mode!=0 || j.kp!=0 || j.ki!=0 || j.kd!=0 || j.tor_max!=0 || j.position!=0)) mark("UNEXPECTED_COMMAND");
  }
  mark(active ? "ACTIVE_COMMAND" : "PASSIVE_COMMAND");
  std::ofstream(std::string(std::getenv("PIANO_XHAND_TEST_DIR"))+"/last_mode") << (active ? "3" : "0");
  if (std::ifstream(std::string(std::getenv("PIANO_XHAND_TEST_DIR"))+"/fail_read"))
    return ErrorStruct(1,"injected device timeout");
  return {};
}
void XHandControl::close_device() { mark("closed"); }
}
