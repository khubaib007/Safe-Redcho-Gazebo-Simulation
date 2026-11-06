"""Isolated, typed, same-cycle single-bank protocol; shared message definitions."""
import json
import math
from . import scenario


def neighbors(agent_id):
    if type(agent_id) is not int or not 1 <= agent_id <= 6:
        raise ValueError('agent_id must be an integer in 1..6')
    return tuple(sorted(((agent_id-2) % 6+1, agent_id % 6+1)))


def topic(suffix):
    return scenario.TOPIC_PREFIX+'/'+suffix


def identity(pid):
    return dict(pid=int(pid), implementation=scenario.IMPLEMENTATION_VERSION,
                architecture=scenario.ARCHITECTURE, banks_per_agent=1)


def parse_identity(detail):
    d = json.loads(detail)
    if (type(d.get('pid')) is not int or d['pid'] <= 0
            or d.get('implementation') != scenario.IMPLEMENTATION_VERSION
            or d.get('architecture') != scenario.ARCHITECTURE
            or type(d.get('banks_per_agent')) is not int or d['banks_per_agent'] != 1):
        raise ValueError('Wrong process identity or single-bank implementation')
    return d


def validate_duration(value):
    if not math.isfinite(value) or value < 0 or abs(value/scenario.DT-round(value/scenario.DT)) > 1e-8:
        raise ValueError('duration must be a nonnegative multiple of 0.01 s')
    return int(round(value/scenario.DT))+1
