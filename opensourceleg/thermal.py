# Thermal Model for the motor devleoped by the M-BLUE team
# @U-M Locomotion Lab, directed by Dr. Robert Gregg
# Authors: Gray C. Thomas, Ph.D, Kevin Best

import numpy as np


class ThermalModel:
    def __init__(
        self,
        ambient=21,
        params=dict(),
        temp_limit_windings=115,
        soft_border_C_windings=15,
        temp_limit_case=80,
        soft_border_C_case=5,
    ):

        # The following parameters result from Jack Schuchmann's test with no fans
        self.C_w = 0.20 * 81.46202695970649
        self.R_WC = 1.0702867186480716
        self.C_c = 512.249065845453
        self.R_CA = 1.9406620046327363
        self.α = 0.393 * 1 / 100  # Pure copper. Taken from thermalmodel3.py
        self.R_T_0 = 65  # temp at which resistance was measured
        self.R_ϕ_0 = 0.376  # emirical, from the computed resistance (q-axis voltage/ q-axis current). Ohms

        self.__dict__.update(params)
        self.T_w = ambient
        self.T_c = ambient
        self.T_a = ambient
        self.soft_max_temp_windings = temp_limit_windings - soft_border_C_windings
        self.abs_max_temp_windings = temp_limit_windings
        self.soft_border_windings = soft_border_C_windings

        self.soft_max_temp_case = temp_limit_case - soft_border_C_case
        self.abs_max_temp_case = temp_limit_case
        self.soft_border_case = soft_border_C_case

    def update(self, dt, I_q_des):
        ## Dynamics:
        # self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
        # self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA

        I2R = (
            I_q_des**2 * self.R_ϕ_0 * (1 + self.α * (self.T_w - self.R_T_0))
        )  # accounts for resistance change due to temp.

        dTw_dt = (I2R + (self.T_c - self.T_w) / self.R_WC) / self.C_w
        dTc_dt = (
            (self.T_w - self.T_c) / self.R_WC + (self.T_a - self.T_c) / self.R_CA
        ) / self.C_c
        self.T_w += dt * dTw_dt
        self.T_c += dt * dTc_dt

    def update_and_get_scale(self, dt, I_q_des, FOS=3.0):
        ## Dynamics:
        # self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
        # self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA

        I2R_des = (
            FOS * I_q_des**2 * self.R_ϕ_0 * (1 + self.α * (self.T_w - self.R_T_0))
        )  # accounts for resistance change due to temp.
        scale = 1.0
        if self.T_w > self.abs_max_temp_windings:
            scale = 0.0
        elif self.T_w > self.soft_max_temp_windings:
            scale *= (self.abs_max_temp_windings - self.T_w) / (
                self.abs_max_temp_windings - self.soft_max_temp_windings
            )

        if self.T_c > self.abs_max_temp_case:
            scale = 0.0
        elif self.T_c > self.soft_max_temp_case:
            scale *= (self.abs_max_temp_case - self.T_w) / (
                self.abs_max_temp_case - self.soft_max_temp_case
            )

        I2R = I2R_des * scale

        dTw_dt = (I2R + (self.T_c - self.T_w) / self.R_WC) / self.C_w
        dTc_dt = (
            (self.T_w - self.T_c) / self.R_WC + (self.T_a - self.T_c) / self.R_CA
        ) / self.C_c
        self.T_w += dt * dTw_dt
        self.T_c += dt * dTc_dt

        if scale <= 0.0:
            return 0.0
        if scale >= 1.0:
            return 1.0

        return np.sqrt(scale)  # this is how much the torque should be scaled
