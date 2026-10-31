"""Decentralized reciprocal HOCBF-QP and Phase-I feasibility recovery."""
import numpy as np
from scipy.optimize import minimize


def quadratic_program(H, f, A, b, lb, ub, initial=None):
    """Solve the same 1/2 x'Hx+f'x objective and hard inequalities as quadprog."""
    if np.any(lb > ub): return None
    x0 = np.clip(-np.linalg.solve(H, f) if initial is None else initial, lb, ub)
    result = minimize(lambda x: 0.5*x@H@x+f@x, x0,
                      jac=lambda x: H@x+f, bounds=list(zip(lb, ub)),
                      constraints=[dict(type='ineq', fun=lambda x: b-A@x, jac=lambda x: -A)],
                      method='SLSQP', options=dict(maxiter=250, ftol=1e-12, disp=False))
    # The MATLAB code also accepts a finite hard-feasible result irrespective of exit flag.
    return result.x if np.all(np.isfinite(result.x)) else None



def solve_decentralized_hocbf_qp(i, u_nom, u_previous, p, v, accel_bound,
                                disturbance_bound, d_safe, p_obs, E_mats,
                                k1_ag, k2_ag, k1_ob, k2_ob, cbf):
    N = p.shape[1]
    anchor = np.clip(u_previous, -accel_bound[None, :], accel_bound[None, :])
    authority = np.zeros(N)
    for candidate in range(N):
        h_min = min((p[:, candidate]-center)@E@(p[:, candidate]-center)-1
                    for center, E in zip(p_obs.T, E_mats))
        authority[candidate] = accel_bound[candidate]*np.clip(h_min/cbf.authority_h_scale, cbf.authority_floor, 1.)
    rows, rhs = [], []
    for j in range(N):
        if j == i: continue
        rp, rv = p[:, i]-p[:, j], v[:, i]-v[:, j]
        h, hd = rp@rp-d_safe**2, 2*rp@rv
        robust = 2*np.linalg.norm(rp)*(disturbance_bound[i]+disturbance_bound[j])
        joint = 2*rv@rv+(k1_ag+k2_ag)*hd+k1_ag*k2_ag*h-robust
        rho = authority[i]/max(authority[i]+authority[j], 1e-9)
        qi, qj = -2*rp@anchor[:, i], 2*rp@anchor[:, j]
        rows.append(-2*rp)
        rhs.append(qi+rho*(joint-qi-qj))
    for center, E in zip(p_obs.T, E_mats):
        delta = p[:, i]-center
        h, hd = delta@E@delta-1, 2*delta@E@v[:, i]
        robust = 2*np.linalg.norm(E@delta)*disturbance_bound[i]
        rows.append(-2*delta@E)
        rhs.append(2*v[:, i]@E@v[:, i]+(k1_ob+k2_ob)*hd+k1_ob*k2_ob*h-robust)
    A, b = np.asarray(rows), np.asarray(rhs)
    norm = np.maximum(np.linalg.norm(A, axis=1), 1e-10)
    A, b = A/norm[:, None], b/norm-cbf.recursive_reserve
    du = cbf.accel_slew_rate*cbf.sample_time
    lb, ub = np.maximum(-accel_bound[i], u_previous[:, i]-du), np.minimum(accel_bound[i], u_previous[:, i]+du)
    minimum_row = np.sum(np.maximum(A, 0)*lb+np.minimum(A, 0)*ub, axis=1)
    info = dict(row_margin=float(np.min(b-minimum_row)),
                recursive_margin=float(min(np.min(anchor[:, i]-lb), np.min(ub-anchor[:, i]), np.min(b-A@anchor[:, i]))))
    H, f = (1+cbf.smoothing_weight)*np.eye(3), -(u_nom+cbf.smoothing_weight*u_previous[:, i])
    u = quadratic_program(H, f, A, b, lb, ub)
    violation = np.inf if u is None else max(0., np.max(A@u-b), np.max(lb-u), np.max(u-ub))
    if violation <= cbf.qp_tolerance:
        info.update(mode=0, phase1_slack=0., max_violation=float(violation))
        return u, info
    # Same 4-variable Phase-I objective. Scale the ENTIRE objective uniformly
    # for SLSQP numerical conditioning; its mathematical minimizer is unchanged.
    Hp = np.diag([1e-8, 1e-8, 1e-8, cbf.phase1_quad_weight])/cbf.phase1_quad_weight
    fp = np.array([0., 0., 0., cbf.phase1_lin_weight])/cbf.phase1_quad_weight
    Ap = np.column_stack([A, -np.ones(len(b))])
    u0 = np.minimum(ub, np.maximum(lb, u_nom))
    z0 = np.r_[u0, max(0., np.max(A@u0-b))]
    z = quadratic_program(Hp, fp, Ap, b, np.r_[lb, 0.], np.r_[ub, np.inf], z0)
    if z is None:
        info.update(mode=1, phase1_slack=np.inf, max_violation=np.inf)
        return u0, info
    u = z[:3]
    info.update(mode=1, phase1_slack=float(max(0., z[3])),
                max_violation=float(max(0., np.max(A@u-b), np.max(lb-u), np.max(u-ub))))
    return u, info

