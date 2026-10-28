"""Lossless legacy evidence retention plus explicit composable migration."""
from copy import deepcopy
from .diagnostics import CompilationError

def import_profile(profile, backend='fake'):
    from morla_core.profile import validate_profile
    if hasattr(profile,'frozen_schema'): validated=profile
    else: validated=validate_profile(deepcopy(profile))
    value=validated.values
    plugins={'nero_arm':'morla.nero.arm','arm':'morla.astral.arm','xhand':'morla.xhand.hand','wuji_hand':'morla.wuji.hand','gripper':'morla.astral.gripper','head':'morla.astral.head','waist':'morla.astral.waist'}
    instances={}; state=[]; action=[]
    for component in value['components']:
        if not component['enabled']: continue
        plugin=plugins.get(component['type'])
        if plugin is None: raise CompilationError('legacy component has no migration descriptor: '+component['type'])
        iid=component['component_id']
        instances[iid]={'plugin':plugin,'version':'1.0.0','role':iid,'backend':backend,'parameters':{'side':component['side']} if component['side'] in {'left','right'} else {}}
        state.append(iid+'.position'); action.append(iid+'.position_target')
    images={}
    for label in value['cameras']:
        iid='camera_'+label.replace('-','_')
        instances[iid]={'plugin':'morla.camera.ros_image','version':'1.0.0','backend':backend}
        images[label]=iid+'.rgb'
    return {'assembly_version':1,'assembly_id':validated.profile_id,'revision':1,'title':value.get('title',validated.profile_id),'instances':instances,'bindings':[],'mounts':[],
            'capture':{'state':state,'action':action,'images':images,'dataset_fps':value['data']['dataset_fps'],'action_source':value['data']['action_source']},
            'legacy':{'profile':deepcopy(value),'frozen_schema':validated.frozen_schema.to_dict(),'schema_hash':validated.frozen_schema.schema_hash,'layout_hash':validated.frozen_schema.layout_hash,'behavior':'exclusive_legacy_lane','diagnostics':[{'error_code':'LEGACY_MIGRATION_REQUIRES_REVIEW','severity':'warning','message':'Legacy stream names, coordinate calibration, driver operations and topic remaps are retained as evidence; composable feature IDs differ. Hardware activation requires deployment-specific review.'}]}}

def legacy_profile_view(resolved):
    source=resolved.get('source_assembly',{})
    legacy=source.get('legacy')
    if not legacy: raise CompilationError('assembly has no lossless legacy profile representation')
    expected=import_profile(legacy['profile'])
    if source!=expected: raise CompilationError('modified migrated assembly cannot be represented losslessly as legacy profile')
    return deepcopy(legacy['profile'])
