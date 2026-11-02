"""Read-only, bounded access to service-owned diagnostic runs."""
import json
from pathlib import Path
import re

_SESSION = re.compile(r'[0-9a-f]{32}')
_FILE = re.compile(r'(?:manifest\.json|summary\.json|trace-[0-9]{5}\.jsonl)')


def run_directory(root, session_id):
    if not isinstance(session_id,str) or not _SESSION.fullmatch(session_id):
        raise ValueError('invalid diagnostic run id')
    root=Path(root).resolve()
    matches=[p for p in root.glob('*_'+session_id) if p.is_dir() and not p.is_symlink()]
    if len(matches)!=1: raise FileNotFoundError('diagnostic run not found')
    result=matches[0].resolve()
    if result.parent!=root: raise ValueError('diagnostic path outside log root')
    return result


def file_path(root, session_id, filename):
    if not _FILE.fullmatch(filename): raise ValueError('invalid diagnostic filename')
    directory=run_directory(root,session_id)
    path=directory/filename
    if path.is_symlink() or not path.is_file(): raise FileNotFoundError('diagnostic file not found')
    if path.resolve().parent!=directory: raise ValueError('diagnostic file outside run')
    return path


def _json(path):
    if path.is_symlink() or path.stat().st_size>8*1024*1024:
        raise ValueError('invalid diagnostic metadata file')
    return json.loads(path.read_text())


def runs(root, active=None, limit=50):
    root=Path(root)
    if not root.is_dir(): return []
    result=[]
    # Bound metadata reads; scan only the log root without recursion.
    directories=sorted((p for p in root.iterdir() if p.is_dir() and not p.is_symlink()),reverse=True)
    for directory in directories[:200]:
        try:
            manifest=_json(directory/'manifest.json')
            sid=manifest['session_id']
            if not _SESSION.fullmatch(sid) or not directory.name.endswith('_'+sid):continue
            files=sorted(p.name for p in directory.iterdir() if _FILE.fullmatch(p.name) and p.is_file() and not p.is_symlink())
            summary=_json(directory/'summary.json') if 'summary.json' in files else {}
            state=(active.diagnostics.status() if active is not None and active.session_id==sid else summary)
            result.append({'session_id':sid,'assembly_id':manifest.get('resolved_system',{}).get('assembly_id'),
                'started_at_utc':manifest.get('started_at_utc'),'state':state.get('state','incomplete'),
                'dropped':state.get('dropped',{}),'written':state.get('written',{}),'error':state.get('error'),
                'files':files,'clean_shutdown':summary.get('clean_shutdown',False)})
        except (OSError,ValueError,KeyError,TypeError): continue
        if len(result)>=limit:break
    return result


def tail(root, session_id, limit=50):
    if type(limit) is not int or not 1<=limit<=200: raise ValueError('tail limit must be 1..200')
    directory=run_directory(root,session_id)
    files=sorted(p.name for p in directory.iterdir() if re.fullmatch(r'trace-[0-9]{5}\.jsonl',p.name) and p.is_file() and not p.is_symlink())
    remaining=1024*1024;records=[];partial=False
    for filename in reversed(files):
        path=file_path(root,session_id,filename)
        with path.open('rb') as stream:
            stream.seek(0,2);size=stream.tell();amount=min(remaining,size);stream.seek(size-amount)
            lines=stream.read(amount).splitlines()
            if amount<size:
                lines=lines[1:];partial=True
        remaining-=amount
        for line in reversed(lines):
            try: records.append(json.loads(line))
            except (ValueError,UnicodeDecodeError):partial=True
            if len(records)>=limit:break
        if len(records)>=limit or remaining<=0:break
    return {'records':list(reversed(records)),'partial':partial,'limit':limit}
