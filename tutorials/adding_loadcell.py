import numpy as np

import opensourceleg.constants as constants
from opensourceleg.osl import OpenSourceLeg

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

osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")
osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=False)
osl.add_joint(name="ankle", gear_ratio=41.99, has_loadcell=False)

osl.add_loadcell(dephy_mode=False, loadcell_matrix=LOADCELL_MATRIX)
