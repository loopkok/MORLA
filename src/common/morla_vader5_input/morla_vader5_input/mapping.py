"""Public control names and one collision-checked mapping; no HID decoder."""
from morla_input.adapters.normalized import InputError

BUTTONS=('UP','RIGHT','DOWN','LEFT','A','B','X','Y','SELECT','START','LB','RB','LT','RT',
         'L3','R3','C','Z','M1','M2','M3','M4','LM','RM','FN','TURBO','HOME')
AXES=('x','y','z','roll','pitch','yaw')
DEFAULT_ACTIONS={'START':'teleop_start','B':'teleop_pause','Y':'record_start','X':'record_stop'}


def configuration(parameters):
    deadman=parameters.get('deadman_button','M1');rotation=parameters.get('rotation_button','M2')
    actions=parameters.get('action_buttons',DEFAULT_ACTIONS)
    if not isinstance(actions,dict):raise InputError('action_buttons must map button to one command')
    if parameters.get('deadzone',.08)<0 or parameters.get('deadzone',.08)>=1:
        raise InputError('Invalid deadzone')
    signs=parameters.get('axis_signs',[1]*6)
    if not isinstance(signs,list) or len(signs)!=6 or any(type(x) not in (int,float) or x not in (-1,1) for x in signs):
        raise InputError('axis_signs must contain six +1/-1 values')
    return deadman,rotation,dict(actions)
