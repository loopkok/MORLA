"""Lazy wrapper of user-supplied JodellTool 0.1.5 (RS485, Modbus RTU)."""
import time
from .protocol import byte,decode,width_to_raw


class EPGTransport:
    def __init__(self,parameters,*,sdk=None):
        self.parameters=parameters;self.port=None;self.master=None;self.closed=False
        if sdk is not None:
            self.sdk=sdk
        else:
            # Do not call SDK.serialOperation: it hard-codes timeout, omits a
            # write timeout and returns error strings instead of raising.
            from jodellSdk.jodellSdkDemo import EpgClawControl
            import serial
            from modbus_tk.modbus_rtu import RtuMaster
            self.sdk=EpgClawControl()
            try:
                self.port=serial.Serial(port=parameters['port'],baudrate=parameters['baudrate'],
                    bytesize=8,parity='N',stopbits=1,timeout=parameters['timeout_s'],
                    write_timeout=parameters['timeout_s'],exclusive=True)
                self.master=RtuMaster(self.port)
                self.master.set_timeout(parameters['timeout_s']);self.master.set_verbose(False)
                self.sdk.setMaster(self.master)
            except Exception:
                if self.port is not None:self.port.close()
                raise
        self.slave=parameters['slave_id']

    @staticmethod
    def _ack(result):
        if type(result) is not int or result!=1:
            raise RuntimeError('EPG SDK write rejected: '+str(result))

    def read(self):
        if self.closed:raise RuntimeError('EPG transport closed')
        raw=self.sdk.readRegisterData(self.slave,4,0x07D0,4)
        sample=decode(raw)  # Strings, truncated replies, uint16 overflow fail closed.
        sample['sample_ns']=time.monotonic_ns()
        return sample

    def enable(self):
        mode=self.sdk.readRegisterData(self.slave,3,0x03FD,1)
        if (not isinstance(mode,(tuple,list)) or len(mode)!=1 or
                type(mode[0]) is not int or not 0<=mode[0]<=65535):
            raise RuntimeError('EPG control mode unreadable')
        if mode[0]&255==0x55:
            raise RuntimeError('EPG is in IO mode; select RS485 mode explicitly before activation')
        self._ack(self.sdk.enableClamp(self.slave,True))

    def write(self,width):
        self._ack(self.sdk.runWithParam(self.slave,width_to_raw(width),
            byte(self.parameters['speed_raw'],'speed_raw'),byte(self.parameters['force_raw'],'force_raw')))

    def stop(self):
        # Manual A.2 pp20-21: rACT=1, rSTOP=1, rGTO=0. SDK.stopClaw uses
        # undocumented 0x03FF; deliberately do not call it or release the brake.
        self._ack(self.sdk.writeDataToRegister(self.slave,0x03E8,[0x0005]))

    def close(self):
        if self.closed:return
        self.closed=True
        try:
            if self.master is not None:self.master.close()
        finally:
            if self.port is not None:self.port.close()
