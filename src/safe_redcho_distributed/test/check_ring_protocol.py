"""Standalone validation of delayed/reordered ring messages and cycle guards."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.ring_protocol import RingInbox,neighbors,payload
agents={i:RingInbox(i) for i in range(1,7)}
assert neighbors(1)==(6,2) and neighbors(6)==(5,1)
# A node can receive its neighbor's publication before its own tick callback.
for cycle in range(1,101):
 for i,a in agents.items():
  for peer in a.neighbors:a.receive(peer,cycle,payload(peer,cycle))
  assert not a.complete() # previous cycle done, next one not begun
 for a in agents.values():
  a.begin(cycle);assert a.complete();assert not a.complete()
 for i,a in agents.items():
  if cycle>1:a.receive(a.neighbors[0],cycle-1,payload(a.neighbors[0],cycle-1))
errors=0
for action in [lambda:agents[1].receive(3,101,payload(3,101)),
               lambda:agents[1].receive(2,101,[2.,100.,102.]),
               lambda:agents[1].receive(2,102,payload(2,102)),
               lambda:agents[1].begin(102),
               lambda:neighbors(0)]:
 try:action()
 except ValueError:errors+=1
assert errors==5
b=RingInbox(1);b.begin(1);b.receive(2,1,payload(2,1));assert not b.complete()
try:b.begin(2)
except ValueError:pass
else:raise AssertionError('Advanced without both neighbors')
print('PASS: six-agent ring, 100 cycles, reordered arrivals, duplicate completion, stale messages and six rejection cases')
