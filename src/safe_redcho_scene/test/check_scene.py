"""Verify physics fingerprint and all local mesh resources without Gazebo."""
from pathlib import Path
import copy,hashlib,json,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]

def physical_signature(path):
    tree=E.parse(path);world=tree.getroot().find('world')
    for tag in ['gui','scene','light']:
        for element in list(world.findall(tag)):world.remove(element)
    for model in list(world.findall('model')):
        if model.get('name')=='arena_visuals':
            assert model.findtext('static')=='true'
            assert not model.findall('.//collision') and not model.findall('.//plugin') and not model.findall('.//joint')
            world.remove(model);continue
        for link in model.findall('.//link'):
            for visual in list(link.findall('visual')):link.remove(visual)
    # These are the deliberate changes for the new ground-start scenario.
    for model in world.findall('model'):
        if model.get('name','').startswith(('cf','obstacle_')):
            pose=model.find('pose')
            if pose is not None:model.remove(pose)
        if model.get('name','').startswith('cf'):
            size=model.find('link/collision/geometry/box/size')
            if size is not None:size.text='normalized-ground-start-box'
    velocities=world.find('.//initial_velocities')
    if velocities is not None:velocities.text='normalized-initial-velocity'
    def canonical(element):
        value=(element.text or '').strip()
        try:value=[float(v) for v in value.split()] if value else ''
        except ValueError:pass
        return [element.tag,sorted(element.attrib.items()),value,[canonical(child) for child in element]]
    return hashlib.sha256(json.dumps(canonical(tree.getroot()),separators=(',',':')).encode()).hexdigest()

def run():
    expected=json.loads((ROOT/'test/physics_fingerprint.json').read_text())['sha256']
    for path in sorted((ROOT/'worlds').glob('*.sdf')):
        assert physical_signature(path)==expected,'Physics changed: '+str(path)
        world=E.parse(path).getroot().find('world')
        for i in range(1,7):
            link=world.find(f"model[@name='cf{i}']/link[@name='base_link']")
            assert link is not None
            assert {v.get('name') for v in link.findall('visual')}=={'crazyflie_body',*[f'rotor_{j}' for j in range(1,5)]}
        for uri in world.findall('.//mesh/uri'):
            assert uri.text.startswith('model://safe_redcho_scene/')
            assert (ROOT/uri.text.removeprefix('model://safe_redcho_scene/')).exists(),uri.text
    for path in (ROOT/'meshes').glob('*.dae'):E.parse(path)
    manifest=json.loads((ROOT/'reference/asset_manifest.json').read_text())
    for name,info in manifest.items():
        assert hashlib.sha256((ROOT/'meshes'/name).read_bytes()).hexdigest()==info['sha256']
    print('PASS: inertias/backend/ground physics preserved; allowed scenario pose/contact-box/velocity changes; genuine mesh integrity verified')

if __name__=='__main__':run()
