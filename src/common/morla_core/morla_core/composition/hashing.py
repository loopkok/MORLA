import hashlib
import json

def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)

def digest(value):
    return hashlib.sha256(canonical_json(value).encode()).hexdigest()
