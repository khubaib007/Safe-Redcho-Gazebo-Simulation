"""Geometric control, motor allocation, and intrinsic SE(3) plant integration."""
import numpy as np


def hat_map(x):
    return np.array([[0., -x[2], x[1]], [x[2], 0., -x[0]], [-x[1], x[0], 0.]])



def vee_map(S): return np.array([S[2, 1], S[0, 2], S[1, 0]])



def so3_exp(x):
    angle = np.linalg.norm(x)
    K = hat_map(x)
    if angle < 1e-8: return np.eye(3)+K+0.5*K@K
    return np.eye(3)+(np.sin(angle)/angle)*K+((1-np.cos(angle))/angle**2)*K@K



def project_to_so3(R):
    U, _, Vh = np.linalg.svd(R)
    return U @ np.diag([1., 1., np.linalg.det(U@Vh)]) @ Vh



def flatness_attitude_from_force(force_world, max_tilt, yaw_des):
    e3 = np.array([0., 0., 1.])
    b3 = force_world/max(np.linalg.norm(force_world), 1e-9)
    tilt = np.arccos(np.clip(b3@e3, -1, 1))
    if tilt > max_tilt:
        horizontal = b3-(b3@e3)*e3
        horizontal /= max(np.linalg.norm(horizontal), 1e-9)
        b3 = np.sin(max_tilt)*horizontal+np.cos(max_tilt)*e3
    b1_heading = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.])
    b2 = np.cross(b3, b1_heading)
    if np.linalg.norm(b2) < 1e-9: b2 = np.cross(b3, np.array([0., 1., 0.]))
    b2 /= max(np.linalg.norm(b2), 1e-9)
    return np.column_stack([np.cross(b2, b3), b2, b3])



def allocation_matrix(params):
    d, c = params.arm_lever, params.CD/params.CT
    return np.array([[1, 1, 1, 1], [-d, -d, d, d], [-d, d, d, -d], [-c, c, -c, c]])



def allocate_motor_thrust(total_thrust, tau_des, params, F_max_motor):
    collective = np.full(4, total_thrust/4)
    delta = np.linalg.solve(allocation_matrix(params), np.r_[0., tau_des])
    delta -= delta.mean()
    alpha = 1.
    for motor in range(4):
        if delta[motor] > 1e-12:
            alpha = min(alpha, (F_max_motor-collective[motor])/delta[motor])
        elif delta[motor] < -1e-12:
            alpha = min(alpha, -collective[motor]/delta[motor])
    return np.clip(collective+np.clip(alpha, 0, 1)*delta, 0, F_max_motor)



def crazyflie_geometric_controller(R, Omega, v, a_des, params, F_max_motor):
    force = params.m*(a_des+np.array([0., 0., params.g]))+params.D_drag*np.linalg.norm(v)*v
    R_des = flatness_attitude_from_force(force, params.max_tilt, 0.)
    e_R = 0.5*vee_map(R_des.T@R-R.T@R_des)
    tau = -params.k_R*e_R-params.k_Omega*Omega+np.cross(Omega, params.inertia@Omega)
    thrust = np.clip(force@R[:, 2], 0, 4*F_max_motor)
    return allocate_motor_thrust(thrust, tau, params, F_max_motor), np.linalg.norm(e_R)



def motor_wrench(motors, params):
    wrench = allocation_matrix(params)@motors
    return wrench[0], wrench[1:]



def translational_acceleration(R, v, total_thrust, params):
    return np.array([0., 0., -params.g])+(total_thrust/params.m)*R[:, 2]-params.D_drag*np.linalg.norm(v)*v/params.m



def integrate_crazyflie_se3(p, v, R, Omega, motors, params, dt):
    thrust, tau = motor_wrench(motors, params)
    a0 = translational_acceleration(R, v, thrust, params)
    od0 = np.linalg.solve(params.inertia, tau-np.cross(Omega, params.inertia@Omega))
    vmid, omid = v+0.5*dt*a0, Omega+0.5*dt*od0
    Rmid = R@so3_exp(0.5*dt*omid)
    amid = translational_acceleration(Rmid, vmid, thrust, params)
    odmid = np.linalg.solve(params.inertia, tau-np.cross(omid, params.inertia@omid))
    return p+dt*vmid, v+dt*amid, project_to_so3(R@so3_exp(dt*omid)), Omega+dt*odmid



def ellipsoid_radial_clearance(position, center, radii):
    delta = position-center
    rho = np.linalg.norm(delta)
    if rho < 1e-12: return -np.min(radii)
    return rho-1/np.sqrt(np.sum((delta/rho/radii)**2))

