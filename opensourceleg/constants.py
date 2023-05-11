from dataclasses import dataclass

import numpy as np


@dataclass
class Constants:
    MOTOR_COUNT_PER_REV = 16384
    NM_PER_AMP = 0.1133
    NM_PER_MILLIAMP = NM_PER_AMP / 1000
    RAD_PER_COUNT = 2 * np.pi / MOTOR_COUNT_PER_REV
    RAD_PER_DEG = np.pi / 180
    MOTOR_COUNT_TO_RADIANS = lambda x: x * (np.pi / 180.0 / 45.5111)
    RADIANS_TO_MOTOR_COUNTS = lambda q: q * (180 * 45.5111 / np.pi)

    RAD_PER_SEC_GYROLSB = np.pi / 180 / 32.8
    M_PER_SEC_SQUARED_ACCLSB = 9.80665 / 8192

    IMPEDANCE_A = 0.00028444
    IMPEDANCE_C = 0.0007812

    NM_PER_RAD_TO_K = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP
    NM_S_PER_RAD_TO_B = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP

    def __repr__(self) -> str:
        return f"""Constants:
                MOTOR_COUNT_PER_REV = {self.MOTOR_COUNT_PER_REV},
                NM_PER_AMP = {self.NM_PER_AMP},
                NM_PER_MILLIAMP = {self.NM_PER_MILLIAMP},
                RAD_PER_COUNT = {self.RAD_PER_COUNT},
                RAD_PER_DEG = {self.RAD_PER_DEG},
                MOTOR_COUNT_TO_RADIANS = {self.MOTOR_COUNT_TO_RADIANS},
                RADIANS_TO_MOTOR_COUNTS = {self.RADIANS_TO_MOTOR_COUNTS},
                RAD_PER_SEC_GYROLSB = {self.RAD_PER_SEC_GYROLSB},
                M_PER_SEC_SQUARED_ACCLSB = {self.M_PER_SEC_SQUARED_ACCLSB},
                IMPEDANCE_A = {self.IMPEDANCE_A},
                IMPEDANCE_C = {self.IMPEDANCE_C},
                NM_PER_RAD_TO_K = {self.NM_PER_RAD_TO_K},
                NM_S_PER_RAD_TO_B = {self.NM_S_PER_RAD_TO_B},
                """
