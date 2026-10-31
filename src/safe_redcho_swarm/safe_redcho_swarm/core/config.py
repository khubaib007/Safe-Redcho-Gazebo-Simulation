"""Simulation parameter classes."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Params:
    m: float = 0.0299
    Ixx: float = 1.395e-5
    Iyy: float = 1.395e-5
    Izz: float = 2.173e-5
    d: float = 0.03973
    CT: float = 3.1582e-10
    CD: float = 7.9379e-12
    F_max_per_motor: float = 0.16
    D_drag: float = 0.012
    g: float = 9.81
    max_tilt: float = np.pi / 6
    c_prop: float = 28.0
    P_avionics: float = 0.85

    @property
    def arm_lever(self): return self.d / np.sqrt(2)
    @property
    def inertia(self): return np.diag([self.Ixx, self.Iyy, self.Izz])
    @property
    def k_R(self): return np.array([2.5e-3, 2.5e-3, 1.1e-3])
    @property
    def k_Omega(self): return np.array([3.1e-4, 3.1e-4, 2.2e-4])



@dataclass
class Battery:
    Q_nom: float = 0.050
    R_int_nom: float = 0.095
    R_p: float = 0.075
    C_p: float = 450.0
    V_nom: float = 4.15
    V_crit: float = 3.0



@dataclass
class CBF:
    max_design_accel: float = 2.80
    smoothing_weight: float = 2.0
    accel_slew_rate: float = 8.0
    sample_time: float = 0.01
    plant_mismatch_bound: float = 0.40
    strict_mismatch_bound: bool = True
    authority_h_scale: float = 3.0
    authority_floor: float = 0.10
    recursive_reserve: float = 0.0
    phase1_quad_weight: float = 1e6
    phase1_lin_weight: float = 1e4
    qp_tolerance: float = 1e-7
    strict_feasibility: bool = True

