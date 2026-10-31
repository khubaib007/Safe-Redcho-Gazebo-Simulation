"""Battery voltage, speed governor, and voltage-dependent acceleration authority."""
import numpy as np


def compute_vocv(soc):
    z = np.clip(soc, 1e-3, 1.0)
    return 3.45 + 0.65*z + 0.10*z*z - 0.05*z**3 - 0.008*np.log(z+1e-3)



def speed_governor(soc, batt):
    chi = (compute_vocv(soc)-batt.V_crit)/(batt.V_nom-batt.V_crit)
    return 0.25 + (0.65-0.25)/(1+np.exp(-6*(chi-0.50)))



def voltage_aware_accel_bound(V, params, batt, a_design_max):
    F = params.F_max_per_motor*np.clip(V/batt.V_nom, 0.5, 1.)**1.5
    aT = 4*F/params.m
    a_tilt = params.g*np.tan(params.max_tilt)/(np.sqrt(2)+np.tan(params.max_tilt))
    disc = 3*aT*aT-2*params.g**2
    a_thrust = 0. if disc <= 0 else max(0., (-params.g+np.sqrt(disc))/3)
    return F, min(a_design_max, a_tilt, a_thrust)

