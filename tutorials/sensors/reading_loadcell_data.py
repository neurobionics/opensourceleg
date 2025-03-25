import numpy as np

from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

LOADCELL_CALIBRATION_MATRIX = np.array([
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
])

if __name__ == "__main__":
    loadcell_logger = Logger(
        log_path="./logs",
        file_name="reading_loadcell_data",
    )
    clock = SoftRealtimeLoop(dt=DT)
    loadcell = DephyLoadcellAmplifier(
        calibration_matrix=LOADCELL_CALIBRATION_MATRIX,
        tag="loadcell",
        amp_gain=125,
        exc=5,
        bus=1,
        i2c_address=102,
    )

    loadcell_logger.track_variable(lambda: loadcell.fx, "Fx")
    loadcell_logger.track_variable(lambda: loadcell.fy, "Fy")
    loadcell_logger.track_variable(lambda: loadcell.fz, "Fz")
    loadcell_logger.track_variable(lambda: loadcell.mx, "Mx")
    loadcell_logger.track_variable(lambda: loadcell.my, "My")
    loadcell_logger.track_variable(lambda: loadcell.mz, "Mz")

    with loadcell:
        for t in clock:
            loadcell.update()
            loadcell_logger.info(
                f"Time: {t}; Fx: {loadcell.fx}; Fy: {loadcell.fy}; Fz: {loadcell.fz};"
                f"Mx: {loadcell.mx}; My: {loadcell.my}; Mz: {loadcell.mz};"
            )
            loadcell_logger.update()
