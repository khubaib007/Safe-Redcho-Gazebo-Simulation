"""Verify published single-bank source, frozen shared equations and worlds."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
original = json.loads((root/'validation/single_source_snapshot.json').read_text())
protected = {p: h for p, h in original.items() if not p.endswith('.md')}
baseline = json.loads((root/'src/safe_redcho_single_sim/provenance/original_source_sha256.json').read_text())
for name in ('config', 'battery', 'reference_filters', 'redcho', 'cbf_qp', 'quadrotor'):
    p = 'src/safe_redcho_swarm/safe_redcho_swarm/core/'+name+'.py'
    protected[p] = baseline[p]
for p in (root/'src/safe_redcho_single_sim/worlds').glob('*.sdf'):
    assert hashlib.sha256(p.read_bytes()).hexdigest() == baseline['src/safe_redcho_scene/worlds/'+p.name]
bad = [p for p, h in protected.items() if not (root/p).is_file() or hashlib.sha256((root/p).read_bytes()).hexdigest() != h]
if bad:
    raise SystemExit('Changed protected source: '+str(bad))
print(json.dumps(dict(status='PASS',protected_files=len(protected),package_runtime_unchanged=True,
                     shared_equations_unchanged=True,worlds_unchanged=True,
                     scope='Published source audit; the original 139-file workstation preservation record is historical'), indent=2))
