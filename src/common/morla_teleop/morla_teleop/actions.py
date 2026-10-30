"""Named operator actions shared by input transports and session hosts."""
def dispatch_input_action(action,request,*,operation,start_recording=None,stop_recording=None):
    operations={'teleop_start':'activate','teleop_pause':'disarm','estop':'estop'}
    if action in operations:return operation(operations[action],request)
    if action=='record_start' and start_recording is not None:return start_recording()
    if action=='record_stop' and stop_recording is not None:return stop_recording()
    raise ValueError('input action has no configured handler: '+action)
