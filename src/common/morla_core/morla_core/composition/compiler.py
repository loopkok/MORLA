"""Pure, deterministic assembly compiler. Does not import or contact devices."""
from copy import deepcopy
import re
from .diagnostics import CompilationError
from .hashing import digest
from .parser import fields, finite_json
from morla_core.plugins.registry import default_catalog
from morla_core.contracts.features import SchemaV3, ActionBlockV3
from morla_core.schema import StreamSpec, StateBlockSpec

_ID = re.compile(r'^[a-z][a-z0-9_-]*$')
_CONTROLLERS={
    'morla.cartesian_delta_ik':({'pose6d'},{'joint_targets'}),
    'morla.gamepad_cartesian_jog':({'axes_buttons'},{'joint_targets'}),
    'morla.joint_jog':({'axes_buttons','joint_targets'},{'joint_targets'}),
    'morla.scalar_map':({'scalar'},{'scalar','joint_targets'}),
    'morla.xhand.retarget':({'hand_landmarks'},{'joint_targets'}),
    'morla.wuji.retarget':({'hand_landmarks'},{'joint_targets'}),
    'morla.identity':({'joint_targets','scalar','numeric'},{'joint_targets','scalar','numeric'}),
}

def _id(value,path):
    if not isinstance(value,str) or not _ID.fullmatch(value): raise CompilationError('expected stable lower-case identifier',path)
    return value

def _rows(value,key):
    if isinstance(value,dict): return [dict(row,**{key:name}) for name,row in sorted(value.items())]
    if isinstance(value,list): return deepcopy(value)
    raise CompilationError('expected mapping or list','/'+key)

def _endpoint(reference,instances,direction=None):
    if not isinstance(reference,str) or '.' not in reference: raise CompilationError('expected instance.port reference',str(reference))
    instance_id,port_id=reference.split('.',1)
    if instance_id not in instances: raise CompilationError('unresolved instance '+instance_id,reference)
    instance=instances[instance_id]
    if port_id not in instance['ports']: raise CompilationError('unresolved port '+reference,reference)
    port=instance['ports'][port_id]
    if direction and port['direction']!=direction: raise CompilationError('port direction mismatch',reference)
    return instance,port_id,port

def compile_system(assembly, deployment=None, registry=None):
    """Return a self-contained JSON snapshot; caller input and catalog remain untouched.

    Snapshot mutation never mutates saved assembly revisions. Consumers must call
    verify_resolved before using a snapshot received across a trust boundary.
    """
    assembly=deepcopy(assembly); deployment=deepcopy(deployment or {})
    finite_json(assembly); finite_json(deployment)
    fields(assembly,{'assembly_version','assembly_id','revision','title','instances','mounts','bindings','control_groups','capture','metadata','legacy'},('assembly_version','assembly_id','revision','instances'))
    if type(assembly['assembly_version']) is not int or assembly['assembly_version']!=1: raise CompilationError('unsupported assembly version','/assembly_version')
    _id(assembly['assembly_id'],'/assembly_id')
    if type(assembly['revision']) is not int or assembly['revision']<1: raise CompilationError('revision must be positive integer','/revision')
    fields(deployment,{'deployment_version','deployment_id','instances','calibrations','allow_mixed_backends','verified','metadata'})
    if 'allow_mixed_backends' in deployment and type(deployment['allow_mixed_backends']) is not bool: raise CompilationError('allow_mixed_backends must be boolean')
    if deployment.get('deployment_version',1)!=1: raise CompilationError('unsupported deployment version')
    catalog=registry or default_catalog()
    instance_bindings=deployment.get('instances',{})
    if not isinstance(instance_bindings,dict): raise CompilationError('deployment instances must be mapping')
    instances={}; sessions={}; diagnostics=[]; locks={}; namespaces=set(); descriptors=[]
    for row in _rows(assembly['instances'],'instance_id'):
        fields(row,{'instance_id','plugin','plugin_id','version','plugin_version','role','namespace','backend','parameters','config','session_id','enabled','display_name'},('instance_id',),'/instances')
        iid=_id(row['instance_id'],'/instances/instance_id')
        if iid in instances: raise CompilationError('duplicate instance id',iid)
        if type(row.get('enabled',True)) is not bool: raise CompilationError('enabled must be boolean',iid)
        if not row.get('enabled',True): continue
        plugin=row.get('plugin',row.get('plugin_id'))
        if not isinstance(plugin,str): raise CompilationError('plugin is required',iid)
        descriptor=catalog.get(plugin,row.get('version',row.get('plugin_version','1.0.0')))
        descriptors.append(descriptor)
        binding=instance_bindings.get(iid,{})
        fields(binding,{'backend','device_id','session_id','resource_key','resources','address','parameters','calibration','verified','stop_verified','serial','bus','secret_ref'},path='/deployment/instances/'+iid)
        backend=binding.get('backend',row.get('backend','fake'))
        if backend not in descriptor['backend_variants']: raise CompilationError('backend not declared by plugin',iid)
        namespace=row.get('namespace','/'+assembly['assembly_id']+'/'+iid)
        if not isinstance(namespace,str) or not re.fullmatch(r'/[a-zA-Z_][a-zA-Z0-9_/]*',namespace) or '//' in namespace or namespace.endswith('/'): raise CompilationError('invalid namespace',iid)
        if namespace in namespaces: raise CompilationError('duplicate namespace',iid)
        namespaces.add(namespace)
        parameters=deepcopy(descriptor.get('defaults',{})); parameters.update(row.get('parameters',row.get('config',{}))); parameters.update(binding.get('parameters',{}))
        if descriptor.get('joint_variants'):
            descriptor['joints']=deepcopy(descriptor['joint_variants'].get(parameters.get('side','left'),descriptor['joints']))
        family=descriptor.get('resources',{}).get('session_family',plugin)
        # Hardware IDs are mandatory for real to avoid accidentally merging unknown boards.
        device_id=binding.get('device_id',binding.get('serial',parameters.get('device_id')))
        if backend=='real' and not device_id: raise CompilationError('real backend requires explicit device_id',iid)
        sid=binding.get('session_id',row.get('session_id')) or (family+'_'+str(device_id) if device_id else iid)
        if not isinstance(sid,str) or not re.fullmatch(r'[a-zA-Z0-9_.:-]+',sid): raise CompilationError('invalid session id',iid)
        resource=binding.get('resource_key',str(device_id) if device_id else 'fake:'+iid)
        resources=binding.get('resources',[resource])
        if not isinstance(resources,list) or any(not isinstance(r,str) or not r for r in resources): raise CompilationError('resources must be nonempty strings',iid)
        for resource_key in resources:
            previous=locks.get(resource_key)
            if previous and previous!=sid: raise CompilationError('exclusive device resource collision: '+resource_key,iid,code='RESOURCE_CONFLICT')
            locks[resource_key]=sid
        if sid in sessions:
            session=sessions[sid]
            if session['session_family']!=family or session['backend']!=backend: raise CompilationError('incompatible logical components share device session',iid)
            if descriptor.get('resources',{}).get('sharing')!='session': raise CompilationError('plugin does not allow sharing device session',iid)
            if binding.get('address') is not None and binding['address'] in session['addresses']: raise CompilationError('duplicate address in shared session',iid)
            session['instances'].append(iid); session['resources']=sorted(set(session['resources']+resources))
            if binding.get('address') is not None: session['addresses'].append(binding['address'])
        else:
            sessions[sid]={'session_id':sid,'session_family':family,'plugin_id':plugin,'backend':backend,'resource_key':resource,'resources':sorted(resources),'instances':[iid], 'addresses':[binding['address']] if 'address' in binding else [], 'parameters':deepcopy(binding.get('parameters',{})),'device_id':device_id,'stop_policy':descriptor.get('resources',{}).get('stop_policy','unsupported')}
        available=bool(descriptor['backend_variants'][backend].get('available',False))
        if descriptor['backend_variants'][backend].get('requires'): diagnostics.append({'error_code':'BACKEND_DEPENDENCIES_UNCHECKED','path':'/instances/'+iid,'severity':'info','message':'Implementation registered; runtime preflight must verify '+', '.join(descriptor['backend_variants'][backend]['requires']),'instances':[iid]})
        if not available: diagnostics.append({'error_code':'BACKEND_UNAVAILABLE','path':'/instances/'+iid,'severity':'warning','message':backend+' backend is not installed/validated','instances':[iid]})
        instances[iid]={'instance_id':iid,'plugin_id':plugin,'plugin_version':descriptor['plugin_version'],'kind':descriptor['kind'],'backend':backend,'namespace':namespace,'role':row.get('role'),'parameters':parameters,'ports':descriptor['ports'],'joints':descriptor.get('joints',[]),'session_id':sid,'resources':resources,'command_modes':descriptor.get('command_modes',[]),'geometry':descriptor.get('geometry',{}),'operations':descriptor.get('operations',{}),'backend_available':available,'stop_policy':descriptor.get('resources',{}).get('stop_policy','unsupported')}
    unknown=set(instance_bindings)-set(instances)
    if unknown: raise CompilationError('deployment references unknown/disabled instances: '+str(sorted(unknown)))
    mounts=assembly.get('mounts',[]); parents={}
    if not isinstance(mounts,list): raise CompilationError('mounts must be list')
    for mount in mounts:
        fields(mount,{'child','parent','calibration','transform'},('child','parent'))
        child,parent=mount['child'],mount['parent']
        for frame in (child,parent):
            if not isinstance(frame,str): raise CompilationError('frame must be string')
            if '.' in frame:
                iid,local=frame.split('.',1)
                if iid not in instances or local not in instances[iid]['geometry'].get('frames',[]): raise CompilationError('unknown geometry frame '+frame)
            elif frame not in {'world','workcell'}: raise CompilationError('unknown root frame '+frame)
        child_i=child.split('.')[0]; parent_i=parent.split('.')[0]
        if child_i in parents: raise CompilationError('mount child has multiple parents',child)
        parents[child_i]=parent_i
        transform=mount.get('transform')
        if transform is not None:
            if not isinstance(transform,list) or len(transform) not in {7,16} or any(type(v) not in {int,float} for v in transform): raise CompilationError('transform must have 7 or 16 finite numbers',child)
            if len(transform)==7 and abs(sum(v*v for v in transform[3:])-1)>1e-5: raise CompilationError('mount quaternion must be normalized',child)
    for node in parents:
        seen=set(); current=node
        while current in parents:
            if current in seen: raise CompilationError('mount cycle',node,code='GEOMETRY_CYCLE')
            seen.add(current); current=parents[current]
    bindings=assembly.get('bindings',[]); ids=set(); target_writers=set()
    if not isinstance(bindings,list): raise CompilationError('bindings must be list')
    for binding in bindings:
        fields(binding,{'id','source','target','controller','parameters','deadman','scope','mode','max_age_ms'},('id','source','target','controller'))
        bid=_id(binding['id'],'/bindings/id')
        if bid in ids: raise CompilationError('duplicate binding id',bid)
        ids.add(bid)
        source,_,sport=_endpoint(binding['source'],instances,'output'); target,_,tport=_endpoint(binding['target'],instances,'input')
        if target['kind']!='actuator': raise CompilationError('binding target must be actuator',bid)
        controller=binding['controller']; accepted=_CONTROLLERS.get(controller)
        if accepted is None:
            desc=catalog.get(controller)
            if desc['kind']!='controller': raise CompilationError('binding controller is not controller plugin',bid)
            descriptors.append(desc)
            types=desc.get('controller_types',{}); accepted=(set(types.get('inputs',[])),set(types.get('outputs',[])))
        if sport['semantic'] not in accepted[0] or tport['semantic'] not in accepted[1]: raise CompilationError('incompatible input/controller/target capability',bid,code='PORT_MISMATCH')
        if controller=='morla.scalar_map' and (sport['dim']!=1 or tport['dim']!=1 or len(target['joints'])!=1):
            raise CompilationError('scalar mapping requires one scalar input and one target joint',bid,code='PORT_MISMATCH')
        if controller=='morla.identity' and (sport['dim']!=tport['dim'] or sport['unit']!=tport['unit']): raise CompilationError('identity mapping requires equal dimensions and units',bid)
        target_identity=(target['instance_id'], 'command')
        if target_identity in target_writers: raise CompilationError('multiple input bindings write same component',bid)
        target_writers.add(target_identity)
    groups=[]; grouped=set(); group_ids=set()
    for group in _rows(assembly.get('control_groups',{}),'group_id'):
        fields(group,{'group_id','members','fault_policy','synchronization','max_dispatch_skew_ms','allow_mixed_backends'},('group_id','members'))
        _id(group['group_id'],'/control_groups')
        if group['group_id'] in group_ids: raise CompilationError('duplicate control group id',group['group_id'])
        group_ids.add(group['group_id'])
        if 'allow_mixed_backends' in group and type(group['allow_mixed_backends']) is not bool: raise CompilationError('allow_mixed_backends must be boolean')
        if not isinstance(group['members'],list) or not group['members'] or len(set(group['members']))!=len(group['members']): raise CompilationError('group needs unique nonempty members')
        for member in group['members']:
            if member not in instances or instances[member]['kind']!='actuator': raise CompilationError('control group member must be known actuator',str(member))
            if member in grouped: raise CompilationError('actuator belongs to multiple groups',member)
            grouped.add(member)
        groups.append(dict(group, fault_policy=group.get('fault_policy','stop_group')))
    for iid,instance in sorted(instances.items()):
        if instance['kind']=='actuator' and iid not in grouped: groups.append({'group_id':iid,'members':[iid],'fault_policy':'stop_group'})
    schema,feature_contract=_compile_capture(assembly,instances)
    geometry={'calibrations':deployment.get('calibrations',{}),'mounts':sorted(mounts,key=lambda m:m['child']),'instances':[{k:instance[k] for k in ('instance_id','plugin_id','joints','geometry')} for instance in instances.values() if instance['kind']!='input']}
    snapshot={'resolved_version':1,'compiler_version':'1.0.0','assembly_id':assembly['assembly_id'],'revision':assembly['revision'],'assembly_revision':assembly['revision'],'assembly_hash':digest(assembly),'deployment_hash':digest(deployment),'feature_contract_hash':digest(feature_contract),'geometry_hash':digest(geometry),'input_mapping_hash':digest(bindings),'instances':list(instances.values()),'device_sessions':[sessions[k] for k in sorted(sessions)],'bindings':bindings,'control_groups':groups,'mounts':mounts,'geometry_contract':geometry,'feature_contract':feature_contract,'capture_schema':schema.to_dict(),'launch_plan':[],'diagnostics':diagnostics,'lockfile':{'compiler_version':'1.0.0','plugins':[[k,v] for k,v in sorted({d['plugin_id']+'@'+d['plugin_version']:digest(d) for d in descriptors}.items())],'codecs':sorted({p.get('codec','numeric') for d in descriptors for p in d['ports'].values()}),'controllers':sorted({b['controller'] for b in bindings})},'status':{'compile_valid':True,'backend_available':all(i['backend_available'] for i in instances.values()),'model_compatible':None},'source_assembly':assembly,'source_deployment':deployment}
    snapshot['resolved_hash']=digest(snapshot)
    return snapshot


def _compile_capture(assembly,instances):
    capture=assembly.get('capture',{})
    fields(capture,{'state','action','images','sensors','dataset_fps','action_source','hold_frames','max_gap_ms','label_source','reference_clock','metadata','auxiliary'})
    state_refs=capture.get('state',[i['instance_id']+'.position' for i in instances.values() if i['kind']=='actuator'])
    action_refs=capture.get('action',[i['instance_id']+'.position_target' for i in instances.values() if i['kind']=='actuator'])
    stream_sources={}; streams={}; blocks={'state':[],'action':[]}; features={'state':[],'action':[]}; cameras={}
    def stream(instance,pid,port,key=None):
        key=key or instance['instance_id']+'__'+pid
        stream_sources[key]={'instance_id':instance['instance_id'],'port_id':pid}
        joint_names=tuple(j['name'] for j in instance['joints']) if len(instance['joints'])==port['dim'] else ()
        streams[key]=StreamSpec(key,instance['namespace']+'/'+pid,'sensor_msgs/msg/CompressedImage' if port['semantic']=='image' else 'sensor_msgs/msg/JointState', (None,None,3) if port['semantic']=='image' else (port['dim'],),port.get('dtype','float64'),port.get('unit','ratio'),'sensor_data','arrival',True,'camera' if port['semantic']=='image' else ('command' if port['direction']=='input' else 'state'),joint_names)
        return key
    for side,refs in (('state',state_refs),('action',action_refs)):
        if not isinstance(refs,list) or len(set(refs))!=len(refs): raise CompilationError('capture references must be unique list','/capture/'+side)
        offset=0
        for reference in refs:
            instance,pid,port=_endpoint(reference,instances)
            if side=='state' and port['direction']!='output': raise CompilationError('state must reference output port',reference)
            if side=='action' and (port['direction']!='input' or instance['kind']!='actuator'): raise CompilationError('action must reference writable actuator input',reference)
            if port['semantic']=='image': raise CompilationError('images belong in capture.images',reference)
            dim=port['dim']; stream_key=stream(instance,pid,port)
            locals_=[j['name'] for j in instance['joints']] if len(instance['joints'])==dim else [f'value{i}' for i in range(dim)]
            names=[instance['instance_id']+'.'+pid+'.'+name for name in locals_]
            if side=='state':
                blocks[side].append(StateBlockSpec(reference,stream_key,dim,tuple(names),port['unit'],tuple(range(offset,offset+dim)),tuple(locals_),(),tuple(range(dim))))
            else:
                state_indexes=[]
                for name in locals_:
                    matches=[idx for idx,f in enumerate(features['state']) if f['instance_id']==instance['instance_id'] and f['local_name']==name and f['port_id']=='position']
                    if len(matches)==1: state_indexes.append(matches[0])
                source_indices=tuple(state_indexes) if len(state_indexes)==dim else ()
                semantics='closure_ratio' if port['unit']=='ratio' else 'absolute_position'
                blocks[side].append(ActionBlockV3(reference,dim,tuple(names),port['unit'],tuple(range(offset,offset+dim)),semantics,True,stream_key,instance['instance_id'],(),source_indices,tuple(range(dim))))
            for name,local in zip(names,locals_):
                features[side].append({'feature_id':name,'name':name,'instance_id':instance['instance_id'],'port_id':pid,'local_name':local,'unit':port['unit'],'semantic':('closure_ratio' if port['unit']=='ratio' else 'absolute_position') if side=='action' else port['semantic']})
            offset+=dim
    images=capture.get('images',{})
    if not isinstance(images,dict): raise CompilationError('capture images must map dataset keys to instance ports')
    for label,reference in sorted(images.items()):
        _id(label,'/capture/images')
        instance,pid,port=_endpoint(reference,instances,'output')
        if port['semantic']!='image': raise CompilationError('camera slot requires image port',reference)
        stream(instance,pid,port,'camera:'+label)
        cameras[label]={'instance_id':instance['instance_id'],'port_id':pid,'dataset_key':label,'unit':'rgb','model_slot':None}
    auxiliary=capture.get('auxiliary',[])
    sensor_specs=[]
    for sensor in capture.get('sensors',[]):
        if isinstance(sensor,str):
            instance,pid,port=_endpoint(sensor,instances,'output'); key=stream(instance,pid,port)
            sensor_specs.append({'sensor_key':key,'source_stream':key,'required':False,'fields':{'values':{'shape':[port['dim']],'dtype':port.get('dtype','float64')}}})
        elif isinstance(sensor,dict):
            fields(sensor,{'sensor_key','source','source_stream','required','fields','unit','codec'},('sensor_key','fields'))
            spec=deepcopy(sensor)
            if 'source' in spec:
                instance,pid,port=_endpoint(spec.pop('source'),instances,'output'); spec['source_stream']=stream(instance,pid,port)
            if spec.get('source_stream') not in streams: raise CompilationError('sensor source stream must resolve to captured output')
            sensor_specs.append(spec)
        else: raise CompilationError('sensor specification must be port reference or mapping')
    if not isinstance(auxiliary,list): raise CompilationError('auxiliary must be a list of port references')
    for reference in auxiliary:
        instance,pid,port=_endpoint(reference,instances,'output'); stream(instance,pid,port)
    action_source={'dispatched_command':'dispatched','authorized_command':'authorized','requested_command':'requested'}.get(capture.get('action_source','command'),capture.get('action_source','command'))
    schema=SchemaV3(assembly['assembly_id'],'composable',tuple(streams[k] for k in sorted(streams)),tuple(blocks['state']),tuple(blocks['action']),tuple(cameras),capture.get('dataset_fps',30),action_source,capture.get('hold_frames',10),capture.get('max_gap_ms',100.0),sensor_specs=tuple(sensor_specs),metadata={'assembly_id':assembly['assembly_id'],'assembly_revision':assembly['revision'],'label_source':capture.get('label_source','dispatched'),'stream_sources':stream_sources})
    return schema,{'state_features':features['state'],'action_features':features['action'],'cameras':cameras,'state_dim':schema.state_dim,'action_dim':schema.action_dim}


def verify_resolved(value, registry=None):
    """Reject tampered payloads before runtime accepts a downloaded artifact."""
    candidate=deepcopy(value); claimed=candidate.pop('resolved_hash',None)
    if not claimed or digest(candidate)!=claimed: raise CompilationError('resolved snapshot hash mismatch',code='RESOLVED_TAMPERED')
    recompiled=compile_system(candidate['source_assembly'],candidate['source_deployment'],registry=registry)
    if recompiled!=value: raise CompilationError('resolved snapshot differs from trusted compiler result',code='RESOLVED_TAMPERED')
    return True
