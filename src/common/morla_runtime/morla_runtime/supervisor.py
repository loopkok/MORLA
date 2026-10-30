"""Process lifecycle shared by installed component factories."""
import os
from importlib import metadata
from .process_executor import ProcessExecutor


class ProcessSupervisor:
    def __init__(self, executor=None, factories=None):
        self.executor=executor or ProcessExecutor()
        self.factories=factories
        self.rows=[]
        self.children={}
        self.specs={}

    def start(self, instances):
        entries=list(metadata.entry_points(group='morla.processes')) if self.factories is None else []
        try:
            for instance in instances:
                iid=instance['instance_id']; plugin=instance['plugin_id']
                matches=[e for e in entries if e.name==plugin]
                available=(plugin in self.factories if self.factories is not None else len(matches)==1)
                if len(matches)>1: raise ValueError('ambiguous process factory: '+plugin)
                mode=instance.get('parameters',{}).get('process_mode','auto')
                if mode=='auto': mode='managed' if available else 'external'
                if mode=='external':
                    self.rows.append({'instance_id':iid,'mode':'external'});continue
                if mode!='managed': raise ValueError('unknown process_mode: '+mode)
                if not available: raise ValueError('missing process factory: '+plugin)
                factory=self.factories[plugin] if self.factories is not None else matches[0].load()
                plan=factory(instance)
                if not plan:self.rows.append({'instance_id':iid,'mode':'in_process'})
                for item in plan:
                    spec=item if isinstance(item,dict) else {'argv':item}
                    argv=spec['argv']; signature=(tuple(argv),tuple(sorted(spec.get('env',{}).items())))
                    key=spec.get('key',signature)
                    if key in self.specs and self.specs[key]!=signature:
                        raise ValueError('shared process key has conflicting configuration: '+str(key))
                    child=self.children.get(key)
                    if child is None:
                        env={**os.environ,**spec['env']} if 'env' in spec else None
                        child=self.executor.start(argv,trusted=True,env=env)
                        self.children[key]=child
                        self.specs[key]=signature
                    self.rows.append({'instance_id':iid,'mode':'managed','child':child,
                                      'readiness':spec.get('readiness','component_feedback')})
        except Exception:
            self.close();raise

    def state(self):
        return [{**{k:v for k,v in row.items() if k!='child'},
                 **({'pid':row['child'].pid,'exit_code':row['child'].poll(),
                     'status':'running' if row['child'].poll() is None else 'exited'}
                    if 'child' in row else {})} for row in self.rows]

    def close(self):
        self.executor.close();self.children.clear();self.specs.clear();self.rows.clear()
