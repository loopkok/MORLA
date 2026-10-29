"""EPG A.2 main register map. No SDK/serial imports in the static contract."""
import math

MODEL='EPG50-060-0PF-L200-C7-N-A-P40-S00'
STROKE_M=.05


def number(value, lower, upper, name):
    if type(value) not in (int,float) or not math.isfinite(value) or not lower<=value<=upper:
        raise ValueError('Invalid '+name)
    return value


def byte(value, name):
    if type(value) is not int or not 0<=value<=255:raise ValueError('Invalid '+name)
    return value


def width_to_raw(width):
    width=number(width,0.,STROKE_M,'opening width (m)')
    return int(math.floor((1-width/STROKE_M)*255+.5))


def decode(registers):
    if not isinstance(registers,(tuple,list)) or len(registers)!=4 or any(
            type(x) is not int or not 0<=x<=65535 for x in registers):
        raise ValueError('EPG requires four uint16 status registers')
    status,position_fault,speed_current,voltage_temperature=registers
    flags=status&255;raw=position_fault>>8
    gto=bool(flags&8);obj=(flags>>6)&3
    return {'width_m':STROKE_M*(1-raw/255), 'closure_ratio':raw/255,
            'position_raw':raw,'fault_code':position_fault&255,
            'enabled':bool(flags&1),'activation_state':(flags>>4)&3,
            'parameter_mode':not bool(flags&4),'go_to':gto,
            'dropped':bool(flags&2),'object_status':obj,
            'object_detected':gto and obj in (1,2),
            'speed_setting_ratio':(speed_current&255)/255,
            'motor_current_ratio':(speed_current>>8)/255,
            'bus_voltage_v':voltage_temperature&255,
            'temperature_c':(voltage_temperature>>8)-256 if voltage_temperature>>8>=128 else voltage_temperature>>8,
            'raw_registers':list(registers)}
