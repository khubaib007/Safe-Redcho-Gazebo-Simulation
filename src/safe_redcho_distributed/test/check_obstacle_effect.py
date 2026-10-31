"""Offline ablation: retain the QP but remove nearby obstacle constraints.
No ROS nodes, flight commands or runtime safety-disable switch are provided.
"""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from safe_redcho_distributed import local_controller as m,scenario
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3
agents={i:m.LocalController(i) for i in range(1,7)}
for agent in agents.values():agent.obstacles=np.array([[1000.,2000.],[1000.,2000.],[1000.,2000.]])
state=m.initial_plant_state();state[0][2]-=9.81e-6;state[1][2]-=.00981;prev=np.zeros((3,6))
for k in range(4501):
 t=k*.01
 prepared={i:a.prepare(k+1,t,state,prev) for i,a in agents.items()};bounds=np.array([prepared[i][1] for i in range(1,7)])
 results={i:a.finish({j:prepared[j][0] for j in a.neighbors},bounds) for i,a in agents.items()}
 p,v,R,w=state
 clearance=np.linalg.norm(p[:,:,None]-scenario.OBSTACLE_CENTERS[:,None,:],axis=0)-scenario.OBSTACLE_RADIUS
 if clearance.min()<scenario.OBSTACLE_CLEARANCE:
  i,j=np.unravel_index(clearance.argmin(),clearance.shape)
  report=dict(status='PASS',meaning='Nearby obstacle constraints are necessary: ablation violates requested clearance',time=float(t),agent=int(i+1),obstacle=int(j+1),clearance=float(clearance.min()),required_clearance=scenario.OBSTACLE_CLEARANCE)
  print(json.dumps(report,indent=2),flush=True)
  if len(sys.argv)>1:Path(sys.argv[1]).write_text(json.dumps(report,indent=2))
  break
 for i in range(6):p[:,i],v[:,i],R[:,:,i],w[:,i]=integrate_crazyflie_se3(p[:,i],v[:,i],R[:,:,i],w[:,i],results[i+1]['u_motors_hist'],m.Params(),.01)
 contact=p[2]<.017425;p[2,contact]=.017425;v[2,contact]=np.maximum(v[2,contact],0.)
 prev=np.stack([results[i]['u_safe_hist'] for i in range(1,7)],axis=1)
else:raise AssertionError('This layout does not demonstrate the obstacle constraint requirement')
