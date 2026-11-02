"""Upgrade only the shipped Vader presets, preserving custom operator maps."""
import copy

VADER_ACTIONS={'M3':'teleop_start','A':'record_or_release','B':'record_stop','R3':'record_discard','X':'workpos'}
LEGACY_ACTIONS=[{'START':'teleop_start','B':'teleop_pause','Y':'record_start','X':'record_stop'},
                {'Z':'teleop_start','B':'teleop_pause','Y':'record_start','X':'record_stop'},
                {'Z':'teleop_start','C':'teleop_pause','Y':'record_start','X':'record_stop'}]


def upgrade(assembly):
    result=copy.deepcopy(assembly)
    for instance in result.get('instances',{}).values():
        if instance.get('plugin')!='morla.input.vader5':continue
        parameters=instance.get('parameters',{})
        actions=parameters.get('action_buttons')
        if actions is None or actions in LEGACY_ACTIONS:
            if actions is not None:parameters['action_buttons']=dict(VADER_ACTIONS)
            if parameters.get('deadman_button')=='M1':parameters['deadman_button']='M3'
    return result
