"""Run from the repository root; validate split package metadata and doc links."""
from pathlib import Path
import ast
import json
import re
import xml.etree.ElementTree as ET

roots = [Path('src/robots/dobot/morla_cr5_driver'),
         Path('src/robots/loophand/morla_loophand_driver')]
for root in roots:
    for path in root.rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    ET.parse(root / 'package.xml')
    descriptor = json.loads((root / 'descriptor.json').read_text())
    assert descriptor['source']['package'] == root.name
    resource = root / 'resource' / (root.name + '_components')
    assert resource.read_text().strip() == f'share/{root.name}/descriptor.json'
for path in [Path('src/common/morla_runtime/morla_runtime/ros_bridge.py'),
             Path('src/common/morla_runtime/morla_runtime/cached_actuator.py')]:
    ast.parse(path.read_text(), filename=str(path))
assert not Path('src/robots/dobot/morla_cr5_loophand').exists()
for root, forbidden in [(roots[0], ['morla_loophand_driver', 'LoopHandTransport', 'pyserial', 'python3-serial']),
                        (roots[1], ['morla_cr5_driver', 'CR5Transport', 'DobotChannel'])]:
    for path in root.rglob('*'):
        if path.suffix in ('.py', '.json', '.xml', '.cfg'):
            assert not any(word in path.read_text() for word in forbidden), str(path)
files = [*Path('docs/integrations').glob('*.md'),
         *(root / 'README.md' for root in roots),
         Path('examples/cr5_loophand/README.md'), Path('docs/device_integration_guide.md')]
count = 0
for path in files:
    for dest in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
        if dest.startswith(('#', 'http:', 'https:')):
            continue
        assert (path.parent / dest.split('#')[0]).exists(), (str(path), dest)
        count += 1
print(f'Python/XML, resource manifests, independent dependencies, removed joint package and {count} local links: PASS')
