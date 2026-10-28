"""Bounded parser properties, including shared/cyclic YAML graphs."""
import json
from pathlib import Path
import tempfile
import pytest
hypothesis=pytest.importorskip('hypothesis')
from hypothesis import given, settings, strategies as st
from morla_core.composition import load_document,CompilationError
from morla_core.composition.parser import finite_json

json_scalar=st.none() | st.booleans() | st.integers(min_value=-10**9,max_value=10**9) | st.floats(allow_nan=False,allow_infinity=False) | st.text(max_size=32)
json_values=st.recursive(json_scalar,lambda child:st.lists(child,max_size=5) | st.dictionaries(st.text(max_size=16),child,max_size=5),max_leaves=30)

@settings(max_examples=80,deadline=None)
@given(json_values)
def test_json_roundtrip_retains_types_and_values(value):
    with tempfile.TemporaryDirectory() as root:
        path=Path(root)/'value.json';path.write_text(json.dumps(value,allow_nan=False))
        assert load_document(path)==value

@settings(max_examples=40,deadline=None)
@given(st.integers(min_value=1,max_value=80))
def test_depth_fuzz_is_bounded(depth):
    with tempfile.TemporaryDirectory() as root:
        path=Path(root)/'depth.yaml';path.write_text('['*depth+'0'+']'*depth)
        if depth>64:
            with pytest.raises(CompilationError):load_document(path)
        else:assert load_document(path) is not None

@settings(max_examples=30,deadline=None)
@given(st.integers(min_value=18,max_value=25))
def test_alias_dag_expansion_cannot_exhaust_hashing(depth):
    rows=['a0: &a0 [0,1]']+[f'a{i}: &a{i} [*a{i-1},*a{i-1}]' for i in range(1,depth)]
    with tempfile.TemporaryDirectory() as root:
        path=Path(root)/'expansion.yaml';path.write_text('\n'.join(rows))
        with pytest.raises(CompilationError):load_document(path)

@given(st.integers(min_value=1,max_value=20))
def test_python_cycle_detected_without_recursion_error(nodes):
    value=[];cursor=value
    for _ in range(nodes):child=[];cursor.append(child);cursor=child
    cursor.append(value)
    with pytest.raises(CompilationError,match='cyclic'):finite_json(value)
