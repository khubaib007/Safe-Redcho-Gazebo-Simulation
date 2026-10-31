"""Cycle matching for the transport test. This is not the REDCHO update law."""
import math


def neighbors(agent_id):
    if agent_id not in range(1,7):
        raise ValueError('agent_id must be in 1..6')
    return ((agent_id-2)%6+1,agent_id%6+1)


def payload(agent_id,cycle):
    return [float(agent_id),float(cycle),float(agent_id+cycle)]


class RingInbox:
    def __init__(self,agent_id):
        self.agent_id=agent_id
        self.neighbors=neighbors(agent_id)
        self.cycle=0
        self.sent=False
        self.messages={peer:{} for peer in self.neighbors}

    def receive(self,peer,cycle,values):
        if peer not in self.neighbors:
            raise ValueError('Message came from a non-neighbor')
        if len(values)!=3 or not all(math.isfinite(x) for x in values):
            raise ValueError('Invalid ring payload')
        if list(values)!=payload(peer,cycle):
            raise ValueError('Neighbor ID/cycle/payload mismatch')
        if cycle<1 or cycle>self.cycle+1:
            raise ValueError('Unexpected future or invalid cycle')
        if cycle<self.cycle:
            return
        self.messages[peer][cycle]=list(values)

    def begin(self,cycle):
        if cycle!=self.cycle+1:
            raise ValueError('Tick sequence is not consecutive')
        if self.cycle and not self.sent:
            raise ValueError('Previous cycle has not completed')
        self.cycle=cycle
        self.sent=False
        for peer in self.neighbors:
            self.messages[peer]={k:v for k,v in self.messages[peer].items() if k>=cycle}

    def complete(self):
        if self.cycle==0 or self.sent:
            return False
        if all(self.cycle in self.messages[peer] for peer in self.neighbors):
            self.sent=True
            return True
        return False
