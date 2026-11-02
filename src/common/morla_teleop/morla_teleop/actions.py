"""Named operator actions shared by input transports and session hosts."""
def dispatch_input_action(action,request,*,operation,start_recording=None,stop_recording=None,
                          discard_recording=None,operator_action=None):
    if operator_action is not None:return operator_action(action,request)
    operations={'teleop_start':'activate','teleop_pause':'disarm','estop':'estop','workpos':'workpos'}
    if action in operations:return operation(operations[action],request)
    if action in ('record_start','record_or_release') and start_recording is not None:return start_recording()
    if action=='record_stop' and stop_recording is not None:return stop_recording()
    if action=='record_discard' and discard_recording is not None:return discard_recording()
    raise ValueError('input action has no configured handler: '+action)
