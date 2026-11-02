"""Bounded, newest-value Quest wire protocol. No ROS or robot dependencies."""
from copy import deepcopy
import math
from .adapters.normalized import InputError, finite

VERSION = 3
MAX_AGE_NS = 250_000_000
MAX_COMMANDS = 32


def integer(value, minimum=0):
    return type(value) is int and minimum <= value <= 2**63-1


def text(value):
    return isinstance(value, str) and 1 <= len(value) <= 128


def validate_values(kind, value):
    def numbers(xs, size, lower=-math.inf, upper=math.inf):
        return (isinstance(xs, list) and len(xs) == size and
                all(type(x) in (int, float) and math.isfinite(x) and lower <= x <= upper for x in xs))
    if kind == 'pose':
        valid = numbers(value, 7) and math.isclose(sum(x*x for x in value[3:]), 1, abs_tol=1e-3)
    elif kind == 'controller_state':
        valid = (isinstance(value, dict) and set(value) == {'trigger','grip','axes','buttons'} and
                 all(type(value[k]) in (int,float) and 0 <= value[k] <= 1 for k in ('trigger','grip')) and
                 numbers(value['axes'], 2, -1, 1) and numbers(value['buttons'], 6, 0, 1) and
                 all(type(x) is int for x in value['buttons']))
    elif kind == 'landmarks':
        valid = isinstance(value, list) and len(value) == 21 and all(numbers(p, 3) for p in value)
        if valid:
            width = math.dist(value[5], value[17])
            valid = .005 <= width <= .20
    else:
        valid = False
    if not valid:
        raise InputError('Invalid Quest snapshot '+kind)


def validate_snapshot(row):
    if isinstance(row,dict) and row.get('version') != VERSION:
        raise InputError('Quest requires snapshot v3; update and restart receiver and Web together')
    required = {'version','kind','epoch','side','source','generation','sequence','sent_ns',
                'basis','frame_id','fields','commands','command_sequence','commands_dropped'}
    if not isinstance(row, dict) or set(row) != required or not finite(row):
        raise InputError('Invalid Quest snapshot envelope')
    if row['version'] != VERSION or row['kind'] != 'snapshot':
        raise InputError('Quest requires snapshot v3; update and restart receiver and Web together')
    if (row['side'] not in ('left','right') or row['source'] not in ('controller','hand') or
            not all(text(row[k]) for k in ('epoch','basis','frame_id')) or
            not all(integer(row[k]) for k in ('sequence','sent_ns','command_sequence','commands_dropped')) or
            not integer(row['generation'], 1)):
        raise InputError('Invalid Quest snapshot identity')
    fields = row['fields']
    allowed = {'pose','controller_state'} if row['source'] == 'controller' else {'pose','landmarks'}
    if not isinstance(fields, dict) or 'pose' not in fields or set(fields)-allowed:
        raise InputError('Invalid Quest snapshot fields')
    for kind, sample in fields.items():
        if (not isinstance(sample, dict) or set(sample) != {'sequence','sent_ns','frame_id','values'} or
                not integer(sample['sequence'], 1) or sample['sequence'] > row['sequence'] or
                not integer(sample['sent_ns']) or sample['sent_ns'] > row['sent_ns'] or not text(sample['frame_id'])):
            raise InputError('Invalid Quest sample identity')
        validate_values(kind, sample['values'])
    if row['frame_id'] != fields['pose']['frame_id']:
        raise InputError('Quest pose frame differs from snapshot frame')
    commands = row['commands']
    if not isinstance(commands, list) or len(commands) > MAX_COMMANDS:
        raise InputError('Quest command journal oversized')
    previous = 0
    actions = {'teleop_start','record_or_release','record_stop','record_discard'} if row['side']=='right' else {'workpos'}
    for command in commands:
        if (not isinstance(command, dict) or set(command) != {'id','sent_ns','action'} or
                not integer(command['id'], 1) or not previous < command['id'] <= row['command_sequence'] or
                not integer(command['sent_ns']) or command['sent_ns'] > row['sent_ns'] or
                command['action'] not in actions or row['source'] != 'controller'):
            raise InputError('Invalid Quest command')
        previous = command['id']


class QuestSnapshotPublisher:
    """Call under the receiver input lock. Never wait for pairing or ACKs.

    Commands remain for 250 ms even after an ACK, so one subscriber cannot
    remove another subscriber's command. ACKs are receipt diagnostics only.
    """
    def __init__(self):
        self.epoch = None
        self.states = {}
        self.acknowledged = {}

    def update(self, *, epoch, side, source, kind, values, frame_id, sent_ns, basis):
        if side not in ('left','right'):
            raise InputError('Invalid Quest side')
        if source not in ('controller','hand'):
            raise InputError('Invalid Quest source')
        validate_values(kind, values)
        if not all(text(x) for x in (epoch,frame_id,basis)) or not integer(sent_ns):
            raise InputError('Invalid Quest publisher identity')
        if epoch != self.epoch:
            self.epoch = epoch; self.states = {}; self.acknowledged = {}
        state = self.states.setdefault(side, {'generation':0,'mode':None,'sequence':0,'fields':{},
                                            'commands':[],'command_sequence':0,'levels':None,'commands_dropped':0})
        if kind == 'pose' and (source != state['mode'] or
                              state['fields'].get('pose',{}).get('frame_id') != frame_id):
            state.update(generation=state['generation']+1, mode=source, fields={},
                         commands=[], command_sequence=0, levels=None)
            self.acknowledged = {k:v for k,v in self.acknowledged.items() if k[1] != side}
        if state['mode'] is None or source != state['mode']:
            return None  # Only a valid pose establishes mode; old buttons cannot change it.
        if kind not in ({'pose','controller_state'} if source == 'controller' else {'pose','landmarks'}):
            raise InputError('Quest sample differs from active mode')
        old = state['fields'].get(kind)
        if old and sent_ns < old['sent_ns']:
            raise InputError('Out-of-order Quest sample')
        state['sequence'] += 1
        state['commands'] = [c for c in state['commands'] if 0 <= sent_ns-c['sent_ns'] <= MAX_AGE_NS]
        if kind == 'controller_state':
            previous = state['levels']
            levels = [values['grip'] > .5, *map(bool,values['buttons'])]
            if previous is not None and old and 0 <= sent_ns-old['sent_ns'] <= MAX_AGE_NS:
                mapping = {0:'teleop_start',1:'record_or_release',2:'record_stop',3:'record_discard'} if side == 'right' else {1:'workpos'}
                # Same priority as the existing operator router: one operation per sample.
                for index, action in mapping.items():
                    if levels[index] and not previous[index]:
                        state['command_sequence'] += 1
                        state['commands'].append({'id':state['command_sequence'],'sent_ns':sent_ns,'action':action})
                        break
            state['levels'] = levels
            if len(state['commands']) > MAX_COMMANDS:
                state['commands_dropped'] += len(state['commands'])-MAX_COMMANDS
                state['commands'] = state['commands'][-MAX_COMMANDS:]
        state['fields'][kind] = {'sequence':state['sequence'],'sent_ns':sent_ns,
                                 'frame_id':frame_id,'values':deepcopy(values)}
        newest = max(s['sent_ns'] for s in state['fields'].values())
        return deepcopy({'version':VERSION,'kind':'snapshot','epoch':epoch,'side':side,'source':source,
                         'generation':state['generation'],'sequence':state['sequence'],'sent_ns':newest,
                         'basis':basis,'frame_id':state['fields']['pose']['frame_id'],
                         'fields':state['fields'],'commands':state['commands'],
                         'command_sequence':state['command_sequence'],'commands_dropped':state['commands_dropped']})

    def acknowledge(self, ack, now_ns):
        if not isinstance(ack, dict) or set(ack) != {'version','epoch','side','generation','command_sequence','consumer'}:
            raise InputError('Invalid Quest acknowledgement')
        if (ack['version'] != VERSION or not text(ack['consumer']) or not integer(ack['command_sequence']) or
                ack['side'] not in ('left','right') or not integer(ack['generation'],1)):
            raise InputError('Invalid Quest acknowledgement')
        state = self.states.get(ack['side'])
        if (ack['epoch'] != self.epoch or state is None or ack['generation'] != state['generation'] or
                ack['command_sequence'] > state['command_sequence']):
            return False
        key = (ack['consumer'],ack['side'])
        previous = self.acknowledged.get(key, (0,0))[0]
        if ack['command_sequence'] < previous:return False
        if key not in self.acknowledged and len(self.acknowledged) >= 16:
            del self.acknowledged[next(iter(self.acknowledged))]
        self.acknowledged[key] = (ack['command_sequence'],now_ns)
        return True
