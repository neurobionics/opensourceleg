from dataclasses import dataclass

import numpy as np

MOTOR_COUNT_PER_REV: float = 16384
NM_PER_AMP: float = 0.1133
NM_PER_MILLIAMP: float = NM_PER_AMP / 1000
RAD_PER_COUNT: float = 2 * np.pi / MOTOR_COUNT_PER_REV
RAD_PER_DEG: float = np.pi / 180

RAD_PER_SEC_GYROLSB: float = np.pi / 180 / 32.8
M_PER_SEC_SQUARED_ACCLSB: float = 9.80665 / 8192

IMPEDANCE_A: float = 0.00028444
IMPEDANCE_C: float = 0.0007812

NM_PER_RAD_TO_K: float = RAD_PER_COUNT / IMPEDANCE_C * 1e3 / NM_PER_AMP
NM_S_PER_RAD_TO_B: float = RAD_PER_DEG / IMPEDANCE_A * 1e3 / NM_PER_AMP

MAX_CASE_TEMPERATURE: float = 80

LOADCELL_MATRIX = np.array(
    [
        (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
        (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
        (
            -1047.16800,
            8.63900,
            -1047.28200,
            -20.70000,
            -1073.08800,
            -8.92300,
        ),
        (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
        (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
        (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
    ]
)
