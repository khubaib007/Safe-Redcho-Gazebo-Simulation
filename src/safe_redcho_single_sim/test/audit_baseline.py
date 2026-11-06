"""Read-only preservation and installed-package audit; output is test evidence."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.workspace.resolve();pkg=root/'src/safe_redcho_single_sim';out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    original=json.loads((pkg/'provenance/original_source_sha256.json').read_text())
    bad=[p for p,h in original.items() if not (root/p).is_file() or digest(root/p)!=h]
    assert not bad,bad
    source=root/'src/safe_redcho_distributed/safe_redcho_distributed'
    text=(pkg/'safe_redcho_single_sim/scenario.py').read_text()
    new="IMPLEMENTATION_VERSION='single-position-redcho-v0.1.0'\nARCHITECTURE='single_position_redcho'\nBANKS_PER_AGENT=1\nTOPIC_PREFIX='/safe_redcho_single_sim'"
    assert text.replace(new,"IMPLEMENTATION_VERSION='redcho-pose-consensus-v2'")==(source/'reactive_scenario.py').read_text()
    pose=(pkg/'safe_redcho_single_sim/pose_redcho.py').read_text().replace('from .protocol import neighbors','from .ring_protocol import neighbors')
    assert pose==(source/'pose_redcho.py').read_text()
    worlds={p.name:digest(p)==original['src/safe_redcho_scene/worlds/'+p.name] for p in (pkg/'worlds').glob('*.sdf')}
    assert all(worlds.values())
    installed=root/'install_single_redcho/safe_redcho_single_sim/lib/python3.12/site-packages/safe_redcho_single_sim'
    modules={p.name:digest(p)==digest(installed/p.name) for p in (pkg/'safe_redcho_single_sim').glob('*.py')}
    assert all(modules.values()),modules
    report=dict(status='PASS',original_files_verified=len(original),scenario_changes_only_version_and_architecture=True,
        position_bank_equations_unchanged=True,worlds_unchanged=worlds,installed_modules_match_source=modules)
    (out/'preservation_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    files=[p for p in pkg.rglob('*') if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts and p.suffix!='.pyc']
    (out/'new_package_sha256.json').write_text(json.dumps({str(p.relative_to(root)):digest(p) for p in sorted(files)},indent=2)+'\n')
    print(json.dumps(dict(status='PASS',original_files=len(original),installed_modules=len(modules))))


if __name__=='__main__':main()
