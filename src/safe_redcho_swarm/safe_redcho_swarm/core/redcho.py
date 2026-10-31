"""Order-2 REDCHO matrices, observer propagation, and perturbation kernel."""
import numpy as np


def build_redcho_matrices(gamma, m_order=2):
    Gamma = np.diag(-gamma)+np.diag(np.ones(m_order), 1)
    C = np.array([1., 0., 0.])
    return Gamma, np.stack([C, C@Gamma, C@Gamma@Gamma])



def redcho_state_derivative(X, Y, adjacency, gamma, theta, gains, L, eps, m_order=2):
    out = np.zeros_like(X)
    for i in range(X.shape[2]):
        injection = np.zeros((3, 3))
        for j in np.flatnonzero(adjacency[i] == 1):
            e = Y[:, 0, i]-Y[:, 0, j]
            for mu in range(m_order+1):
                alpha, beta = (m_order-mu)/(m_order+1), (mu+1)/(m_order+1)
                signed_power = e*(e*e+eps*eps)**((alpha-1)/2) if eps > 0 else np.abs(e)**alpha*np.sign(e)
                injection[:, mu] += gains[mu]*theta**(mu+1)*L**beta*signed_power
        out[:, :, i] = -X[:, :, i]*gamma[None, :]+injection
        out[:, :2, i] += X[:, 1:, i]
    return out



def redcho_strict_kernel(t, omega, c_w):
    x = omega*np.asarray(t)
    ex = np.exp(-x)
    return c_w[0]*omega*x*x/2*ex+c_w[1]*omega**2*(x-x*x/2)*ex+c_w[2]*omega**3*(1-2*x+x*x/2)*ex

