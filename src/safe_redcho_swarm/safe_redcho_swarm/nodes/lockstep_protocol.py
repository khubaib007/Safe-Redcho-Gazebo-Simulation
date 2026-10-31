"""Checks for an atomic six-agent Gazebo response; no ROS imports."""
import numpy as np
from scipy.spatial.transform import Rotation

N = 6
CONTROL_NS = 10_000_000


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_reply(reply, sequence, previous_ns, commanded, limits, *, expected_steps=10, expected_wrenches=10, expected_ns=CONTROL_NS):
    if not reply.success:
        raise RuntimeError('Gazebo rejected the step: ' + reply.status)
    if int(reply.sequence) != sequence:
        raise RuntimeError('Response sequence mismatch')
    if reply.physics_steps != expected_steps or reply.wrench_steps != expected_wrenches:
        raise RuntimeError('Unexpected physics/wrench counts')
    ns = stamp_ns(reply.stamp)
    if ns - previous_ns != expected_ns:
        raise RuntimeError(f'Expected {expected_ns} ns step, received {ns-previous_ns} ns')
    if len(reply.states) != N:
        raise RuntimeError('Expected exactly six states')
    actual = np.asarray(reply.applied_forces, dtype=float)
    caps = np.asarray(reply.applied_limits, dtype=float)
    if actual.shape != (4*N,) or caps.shape != (N,):
        raise RuntimeError('Applied motor batch shape mismatch')
    expected = np.clip(np.asarray(commanded).reshape(N,4), 0,
                       np.asarray(limits).reshape(N,1))
    if (not np.all(np.isfinite(actual)) or
            not np.allclose(actual.reshape(N,4), expected, rtol=0, atol=1e-12) or
            not np.allclose(caps, limits, rtol=0, atol=1e-12)):
        raise RuntimeError('Applied forces/caps do not match the requested batch')
    positions, velocities, matrices, omegas = [], [], [], []
    for i, state in enumerate(reply.states):
        if (stamp_ns(state.header.stamp) != ns or
                state.header.frame_id != 'world' or
                state.child_frame_id != f'cf{i+1}/base_link'):
            raise RuntimeError('State timestamp/frame/order mismatch')
        p = state.pose.pose.position
        q = state.pose.pose.orientation
        v = state.twist.twist.linear
        w = state.twist.twist.angular
        pos = np.array([p.x,p.y,p.z])
        quat = np.array([q.x,q.y,q.z,q.w])
        vb = np.array([v.x,v.y,v.z])
        wb = np.array([w.x,w.y,w.z])
        if (not np.all(np.isfinite(np.r_[pos,quat,vb,wb])) or
                abs(np.linalg.norm(quat)-1.0) > 1e-3):
            raise RuntimeError('Invalid state or non-unit quaternion')
        R = Rotation.from_quat(quat).as_matrix()
        positions.append(pos)
        velocities.append(R @ vb)  # Odometry child-frame twist -> world velocity
        matrices.append(R)
        omegas.append(wb)
    return ns, np.array(positions), np.array(velocities), np.array(matrices), np.array(omegas)
