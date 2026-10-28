"""morla-system commands are pure until a separate runtime session is requested."""
import argparse
import json
import os
from pathlib import Path
import tempfile
from . import compile_system, default_catalog, import_profile, load_document, CompilationError

def write_atomic(path,value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    descriptor,name=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(descriptor,'w',encoding='utf-8') as stream:
            json.dump(value,stream,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False); stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(name,path)
    finally:
        if os.path.exists(name): os.unlink(name)

def main(argv=None):
    parser=argparse.ArgumentParser(prog='morla-system')
    parser.add_argument('--json',action='store_true')
    subs=parser.add_subparsers(dest='command',required=True)
    subs.add_parser('catalog').add_argument('--json',action='store_true')
    for command in ('validate','compile'):
        sub=subs.add_parser(command); sub.add_argument('assembly'); sub.add_argument('--deployment'); sub.add_argument('--output'); sub.add_argument('--json',action='store_true')
    sub=subs.add_parser('diff'); sub.add_argument('before');sub.add_argument('after');sub.add_argument('--json',action='store_true')
    sub=subs.add_parser('import-profile');sub.add_argument('profile');sub.add_argument('--output');sub.add_argument('--json',action='store_true')
    args=parser.parse_args(argv)
    try:
        if args.command=='catalog': result=default_catalog().to_dict()
        elif args.command=='import-profile': result=import_profile(load_document(args.profile))
        elif args.command=='diff':
            before,after=load_document(args.before),load_document(args.after)
            result={'changed_fields':[key for key in sorted(set(before)|set(after)) if before.get(key)!=after.get(key)]}
        else:
            result=compile_system(load_document(args.assembly),load_document(args.deployment) if args.deployment else None)
            if args.command=='validate': result={'valid':True,'diagnostics':result['diagnostics'],'status':result['status'],'resolved_hash':result['resolved_hash']}
        if getattr(args,'output',None): write_atomic(args.output,result)
        print(json.dumps(result,sort_keys=True,ensure_ascii=False,indent=2,allow_nan=False))
        return 0
    except (ValueError,OSError,TypeError,KeyError) as exc:
        print(json.dumps({'valid':False,'diagnostics':getattr(exc,'diagnostics',[{'error_code':'INVALID_CONFIGURATION','path':'','message':str(exc),'severity':'error'}])},ensure_ascii=False))
        return 2

if __name__=='__main__': raise SystemExit(main())
