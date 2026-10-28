"""Aero Hand Open serial protocol; no automatic homing or enabling."""
import math
import struct


class LoopHandTransport:
    def __init__(self, parameters):
        from serial import Serial
        from .vendor.aero_hand_constants import AeroHandConstants
        from .vendor.joints_to_actuations import JointsToActuationsModel
        from .vendor.actuations_to_joints import ActuationsToJointsModelCompact
        path = parameters['port']
        if not path.startswith('/dev/serial/by-id/'):
            raise ValueError('LoopHand requires /dev/serial/by-id/ identity')
        self.serial = Serial(path, 921600, timeout=parameters['timeout_s'],
                             write_timeout=parameters['timeout_s'], exclusive=True)
        self.constants = AeroHandConstants()
        self.forward = JointsToActuationsModel()
        self.inverse = ActuationsToJointsModelCompact()

    def send(self, opcode, values):
        packet = struct.pack('<2B7H', opcode, 0, *values)
        if self.serial.write(packet) != len(packet):
            raise IOError('Partial LoopHand write')

    def read(self):
        self.serial.reset_input_buffer()
        self.send(0x22, [0]*7)
        packet = self.serial.read(16)
        if len(packet) != 16 or packet[:2] != bytes([0x22, 0]):
            raise IOError('Invalid/timeout LoopHand GET_POS response')
        raw = struct.unpack('<2B7H', packet)[2:]
        c = self.constants
        actuations = [math.radians(lo + value/65535*(hi-lo)) for value,lo,hi in
                      zip(raw,c.actuation_lower_limits,c.actuation_upper_limits)]
        # Estimated anatomical angles from measured actuator encoders, never 16 measured joints.
        return list(self.inverse.hand_joints(actuations))

    def activate(self):
        # No firmware enable opcode; no homing or torque changes on activation.
        pass

    def write(self, values):
        a,b,c,d,e,f,g = map(math.degrees, values)
        actuations = self.forward.hand_actuations([a,b,c,c,d,d,d,e,e,e,f,f,f,g,g,g])
        output=[]
        for value,lo,hi in zip(actuations,self.constants.actuation_lower_limits,self.constants.actuation_upper_limits):
            if not lo-1e-6 <= value <= hi+1e-6:
                raise ValueError('Coupled tendon target exceeds actuator range')
            output.append(round(max(0,min(65535,(value-lo)/(hi-lo)*65535))))
        self.send(0x11,output)

    def stop(self):
        self.send(0x12,[0]*7)  # Firmware torque-zero request; no execution ACK exists.
        raise RuntimeError('LOOPHAND_STOP_UNCONFIRMED: firmware provides no torque-zero ACK')

    def close(self):
        self.serial.close()
