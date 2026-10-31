"""Check numerical imports, syntax, package XML and scene consistency; no ROS."""
import ast,json,sys,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy,scipy,matplotlib
from safe_redcho_distributed import scenario
from safe_redcho_distributed.local_controller import LocalController
from safe_redcho_scene.geometry_guard import validate_world_geometry
root=Path(__file__).resolve().parents[1]
for p in (root/'src').rglob('*.py'): ast.parse(p.read_text(),filename=str(p))
manifests=list((root/'src').glob('*/package.xml'))
for p in manifests: ET.parse(p)
for p in (root/'scripts').glob('*.sh'): subprocess.run(['bash','-n',str(p)],check=True)
for p in (root/'src/safe_redcho_scene/worlds').glob('*.sdf'): validate_world_geometry(p,scenario)
LocalController(1)
print(json.dumps(dict(status='PASS_LOCAL_SOFTWARE',variant=json.loads((root/'release.json').read_text())['variant'],packages=len(manifests),python=sys.version.split()[0],numpy=numpy.__version__,scipy=scipy.__version__,matplotlib=matplotlib.__version__,scope='No ROS graph, native Gazebo or hardware execution'),indent=2))
