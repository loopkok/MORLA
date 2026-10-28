"""Bounded strict YAML/JSON loader; duplicate keys and cyclic aliases are errors."""
import math
import json
from .diagnostics import CompilationError

MAX_DOCUMENT_BYTES=1_000_000
MAX_DEPTH=64
MAX_ALIASES=128
MAX_EXPANDED_NODES=100_000


def finite_json(value,path=''):
    """Validate finite JSON types, graph acyclicity and bounded expanded size.

    Noncyclic shared legacy YAML anchors remain valid. Subtree size memoization
    counts every alias occurrence without exponentially visiting the same graph.
    """
    active=set();sizes={}
    def visit(item,pointer,depth):
        if depth>MAX_DEPTH:raise CompilationError('configuration exceeds maximum nesting depth',pointer)
        if type(item) is float and not math.isfinite(item):raise CompilationError('non-finite number',pointer)
        if isinstance(item,(dict,list,tuple)):
            identity=id(item)
            if identity in active:raise CompilationError('cyclic configuration alias',pointer)
            if identity in sizes:return sizes[identity]
            active.add(identity);size=1
            children=item.items() if isinstance(item,dict) else enumerate(item)
            for key,child in children:
                if isinstance(item,dict) and type(key) is not str:raise CompilationError('mapping keys must be strings',pointer)
                size+=visit(child,pointer+'/'+str(key),depth+1)
                if size>MAX_EXPANDED_NODES:raise CompilationError('configuration expanded node limit exceeded',pointer)
            active.remove(identity);sizes[identity]=size;return size
        if item is not None and type(item) not in (str,int,float,bool):raise CompilationError('value is not JSON compatible',pointer)
        return 1
    visit(value,path,0)
    return value


def load_document(path):
    import yaml
    with open(path,'rb') as stream:raw=stream.read(MAX_DOCUMENT_BYTES+1)
    if len(raw)>MAX_DOCUMENT_BYTES:raise CompilationError('configuration exceeds maximum document size')
    try:
        content=raw.decode('utf-8')
        depth=aliases=0
        for event in yaml.parse(content,Loader=yaml.SafeLoader):
            if isinstance(event,(yaml.MappingStartEvent,yaml.SequenceStartEvent)):
                depth+=1
                if depth>MAX_DEPTH:raise CompilationError('configuration exceeds maximum nesting depth')
            elif isinstance(event,(yaml.MappingEndEvent,yaml.SequenceEndEvent)):depth-=1
            elif isinstance(event,yaml.AliasEvent):
                aliases+=1
                if aliases>MAX_ALIASES:raise CompilationError('configuration exceeds maximum YAML alias count')
        class StrictLoader(yaml.SafeLoader):pass
        def mapping(loader,node,deep=False):
            result={}
            for key_node,value_node in node.value:
                key=loader.construct_object(key_node,deep=deep)
                if type(key) is not str:raise CompilationError('mapping keys must be strings')
                if key in result:raise CompilationError('duplicate key: '+key)
                result[key]=loader.construct_object(value_node,deep=deep)
            return result
        StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
        if str(path).lower().endswith('.json'):
            def json_pairs(pairs):
                result={}
                for key,child in pairs:
                    if key in result:raise CompilationError('duplicate key: '+key)
                    result[key]=child
                return result
            def invalid_constant(value):raise CompilationError('non-finite JSON constant: '+value)
            value=json.loads(content,object_pairs_hook=json_pairs,parse_constant=invalid_constant)
        else:
            value=yaml.load(content,Loader=StrictLoader)
    except (yaml.YAMLError,json.JSONDecodeError,UnicodeError,RecursionError) as exc:raise CompilationError('invalid bounded YAML/JSON document: '+str(exc)) from exc
    return finite_json(value)


def fields(value,allowed,required=(),path=''):
    if not isinstance(value,dict):raise CompilationError('expected object',path)
    extra,missing=set(value)-set(allowed),set(required)-set(value)
    if extra or missing:raise CompilationError(f'unknown={sorted(extra)}, missing={sorted(missing)}',path)
