"""Reusable adapter ABI harness. Structural checks never open/configure devices."""
import inspect


def validate_adapter_contract(adapter,instance):
    calls={'configure':({'session_id':'structural-check','instance':instance},),'health':(1,),'close':()}
    if instance.get('kind')=='actuator':calls.update(prepare=(),activate=(),submit=({'names':[],'values':[]},),stop=())
    if callable(getattr(adapter,'read',None)):calls['read']=(1,)
    unchecked=[]
    for method,arguments in calls.items():
        fn=getattr(adapter,method,None)
        if not callable(fn):raise TypeError('ADAPTER_PROTOCOL_INCOMPLETE: '+method)
        try:signature=inspect.signature(fn)
        except (TypeError,ValueError):unchecked.append(method);continue
        try:signature.bind(*arguments)
        except TypeError as exc:raise TypeError('ADAPTER_SIGNATURE_MISMATCH: '+method) from exc
    return {'valid':True,'methods':sorted(calls),'uninspectable_methods':unchecked,'hardware_touched':False}


def exercise_fake_adapter(adapter,instance):
    """Explicit behavioral probe is restricted to declared fake backends."""
    if getattr(adapter,'backend',None)!='fake' or instance.get('backend')!='fake':raise ValueError('behavioral conformance harness is fake-only; physical acceptance is separate')
    validate_adapter_contract(adapter,instance)
    adapter.configure({'session_id':'conformance','instance':instance})
    try:
        if not adapter.health(1).get('ready'):raise AssertionError('fake adapter not ready after configure')
        if instance.get('kind')=='actuator':
            adapter.prepare();adapter.activate()
            names=[joint['name'] for joint in instance['joints']]
            values=[max(joint['lower'],min(0.0,joint['upper'])) for joint in instance['joints']]
            receipt=adapter.submit({'component_id':instance['instance_id'],'names':names,'values':values,'mode':'position','units':[joint['unit'] for joint in instance['joints']]})
            if not receipt.get('accepted'):raise AssertionError('fake adapter rejected reference command')
            adapter.stop()
        return {'valid':True,'backend':'fake','hardware_touched':False}
    finally:adapter.close()


def main(argv=None):
    """Offline installed-plugin harness; refuses any nonfake assembly backend."""
    import argparse
    import json
    import tempfile
    from morla_core.composition import compile_system,load_document
    from morla_runtime import RuntimeSession
    parser=argparse.ArgumentParser(description='Run the MORLA installed fake-adapter conformance harness')
    parser.add_argument('assembly');parser.add_argument('--instance',required=True);parser.add_argument('--exercise-fake',action='store_true')
    args=parser.parse_args(argv)
    resolved=compile_system(load_document(args.assembly))
    if any(instance['backend']!='fake' for instance in resolved['instances']):raise ValueError('standalone conformance harness refuses nonfake backends')
    instance=next((row for row in resolved['instances'] if row['instance_id']==args.instance),None)
    if instance is None:raise ValueError('instance not found')
    with tempfile.TemporaryDirectory(prefix='morla-conformance-') as root:
        session=RuntimeSession(resolved,resource_root=root)
        try:
            session.start();adapter=session.adapters[args.instance]
            result=exercise_fake_adapter(adapter,instance) if args.exercise_fake else validate_adapter_contract(adapter,instance)
            print(json.dumps(result,sort_keys=True))
        finally:session.stop()
    return 0

if __name__=='__main__':raise SystemExit(main())
