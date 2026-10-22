"""Real SDK packet encoding intercepted before transport; never opens a device."""
import os
from pathlib import Path
import subprocess

import pytest

pytestmark=pytest.mark.skipif(not os.environ.get('MORLA_TEST_XHAND_INCLUDE'),reason='explicit native SDK test')


def test_real_sdk_passive_packet_is_command_2_and_zeroes_reserved_words(tmp_path):
    include=Path(os.environ['MORLA_TEST_XHAND_INCLUDE'])
    library=include.parent/'lib'
    # The transport header stores only shared_ptr<Serial>; the vendor serial
    # implementation is binary-only. No Serial method is used by this test.
    header=(include/'serial_communication.hpp').read_text().replace('#include "serial/serial.h"','namespace serial { class Serial; }')
    (tmp_path/'serial_communication.hpp').write_text(header)
    shim=tmp_path/'shim.cpp'
    shim.write_text('''
#include "xhand_control.hpp"
#include "serial_communication.hpp"
#include <fstream>
#include <cstdlib>
namespace xhand_control {
ErrorStruct XHandControl::open_serial(const std::string&,uint32_t) {
 device_=std::make_shared<serial_communication::SerialCommunication>(); return {};
}
void XHandControl::close_device() { device_.reset(); }
}
namespace serial_communication {
bool SerialCommunication::package_and_write(uint8_t,uint8_t cmd,const std::vector<uint8_t>& data,
    xhand_control::ErrorStructPtr,uint16_t,uint32_t) {
 std::ofstream out(std::getenv("PACKET_FILE"),std::ios::binary); out.put(cmd);
 out.write(reinterpret_cast<const char*>(data.data()),data.size()); return true;
}
void SerialCommunication::close_device() {}
}
''')
    main=tmp_path/'main.cpp'
    main.write_text('''
#include "xhand_control.hpp"
int main(){
 xhand_control::XHandControl hand;
 if(!hand.open_serial("/dev/DOES-NOT-EXIST-protocol-test",3000000))return 1;
 HandCommand_t command{};
 for(int i=0;i<12;i++)command.finger_command[i].id=i;
 auto result=hand.send_command(0,command); hand.close_device();return result?0:2;
}
''')
    shared=tmp_path/'intercept.so';exe=tmp_path/'encode'
    subprocess.run(['g++','-shared','-fPIC','-std=c++14','-I'+str(tmp_path),'-I'+str(include),str(shim),'-o',str(shared)],check=True)
    subprocess.run(['g++','-std=c++14','-I'+str(include),str(main),'-L'+str(library),'-lxhand_control',
                    '-Wl,-rpath,'+str(library),'-o',str(exe)],check=True)
    packet=tmp_path/'packet'
    subprocess.run([str(exe)],env=dict(os.environ,LD_PRELOAD=str(shared),PACKET_FILE=str(packet)),timeout=5,check=True,capture_output=True)
    raw=packet.read_bytes()
    assert raw[0]==2 and len(raw)==289  # no command 5 / flash operation
    import struct
    for i in range(12):
        words=struct.unpack('<12H',raw[1+i*24:1+(i+1)*24])
        assert words[0]==i and all(word==0 for word in words[1:])
