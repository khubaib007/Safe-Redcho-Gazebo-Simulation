"""State-variable-filter certification and reference regularization."""
import numpy as np
from scipy.integrate import quad


def compute_svf_iss_certificate(omega, raw_input_bound, initial_output_bound):
    gain = 1+quad(lambda x: np.exp(-x)*abs(-3+3*x-0.5*x*x), 0, np.inf)[0]
    return dict(forced_gain=gain, initial_gain=1., L_svf_vector=omega**3*(gain*raw_input_bound+initial_output_bound))




def svf_step(Z, raw, omega, dt):
    """Explicit-midpoint third-order SVF update; return states and snap."""
    def svf_dot(z):
        zd = np.empty_like(z)
        zd[:, 0, :], zd[:, 1, :] = z[:, 1, :], z[:, 2, :]
        zd[:, 2, :] = -omega**3*(z[:, 0, :]-raw)-3*omega**2*z[:, 1, :]-3*omega*z[:, 2, :]
        return zd
    Z = Z+dt*svf_dot(Z+.5*dt*svf_dot(Z))
    snap = -omega**3*(Z[:, 0, :]-raw)-3*omega**2*Z[:, 1, :]-3*omega*Z[:, 2, :]
    return Z, snap


def levant_step(Z_levant, raw, L, dt):
    """Explicit-Euler Levant ablation update with the source's gains."""
    elev = Z_levant[:, 0, :]-raw
    ld = np.zeros_like(Z_levant)
    ld[:, 0, :] = Z_levant[:, 1, :]-2*L**(1/3)*np.abs(elev)**(2/3)*np.sign(elev)
    ld[:, 1, :] = Z_levant[:, 2, :]-1.5*L**(2/3)*np.abs(elev)**(1/3)*np.sign(elev)
    ld[:, 2, :] = -1.1*L*np.sign(elev)
    return Z_levant+dt*ld
