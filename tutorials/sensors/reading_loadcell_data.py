import numpy as np
from opensourceleg_rs import Logger

from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.sensors.loadcell import DephyLoadcellAmplifier, NBLoadcellDAQ
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

LOADCELL_CALIBRATION_MATRIX_M3564F = np.array([
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

LOADCELL_CALIBRATION_MATRIX_M3554E = np.array([
    (-943.401, 4.143, 8.825, -16.57, 952.216, 10.892),
    (539.853, 14.985, -1111.656, -0.812, 546.9, -18.949),
    (13.155, 533.082, -4.582, 534.843, 10.827, 536.327),
    (0.138, -10.419, 0.202, 0.14, 0.063, 10.518),
    (-0.075, 6.213, -0.239, -12.094, 0.181, 6.156),
    (-19.912, 0.082, -20.347, 0.022, -19.486, 0.013),
])


def demo_loadcell_i2c(clock):
    loadcell = DephyLoadcellAmplifier(
        calibration_matrix=LOADCELL_CALIBRATION_MATRIX_M3564F,
        tag="loadcell",
        amp_gain=125,
        exc=5,
        bus=1,
        i2c_address=102,
    )

    Logger.track_functions({
        "Fx": lambda: loadcell.fx,
        "Fy": lambda: loadcell.fy,
        "Fz": lambda: loadcell.fz,
        "Mx": lambda: loadcell.mx,
        "My": lambda: loadcell.my,
        "Mz": lambda: loadcell.mz,
    })

    with loadcell:
        loadcell.calibrate(reset=True)
        for t in clock:
            loadcell.update()
            Logger.info(
                f"Time: {t}; Fx: {loadcell.fx}; Fy: {loadcell.fy}; Fz: {loadcell.fz};"
                f"Mx: {loadcell.mx}; My: {loadcell.my}; Mz: {loadcell.mz};"
            )
            Logger.record()


def demo_loadcell_actpack(clock):
    actpack = DephyActuator(
        port="/dev/ttyACM1",
        gear_ratio=9,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )

    loadcell = DephyLoadcellAmplifier(
        calibration_matrix=LOADCELL_CALIBRATION_MATRIX_M3564F,
        tag="loadcell",
        amp_gain=125,
        exc=5,
    )

    Logger.track_functions({
        "Fx": lambda: loadcell.fx,
        "Fy": lambda: loadcell.fy,
        "Fz": lambda: loadcell.fz,
        "Mx": lambda: loadcell.mx,
        "My": lambda: loadcell.my,
        "Mz": lambda: loadcell.mz,
    })

    def get_raw_loadcell_actpack():
        actpack.update()
        return actpack.genvars

    with actpack, loadcell:
        loadcell.calibrate(reset=True, data_callback=get_raw_loadcell_actpack)
        for t in clock:
            loadcell.update(data_callback=get_raw_loadcell_actpack)
            Logger.info(
                f"Time: {t}; Fx: {loadcell.fx}; Fy: {loadcell.fy}; Fz: {loadcell.fz};"
                f"Mx: {loadcell.mx}; My: {loadcell.my}; Mz: {loadcell.mz};"
            )
            Logger.update()


def demo_loadcell_nb_daq(clock):
    loadcell = NBLoadcellDAQ(
        LOADCELL_CALIBRATION_MATRIX_M3554E,
        tag="loadcell",
        excitation_voltage=5.0,
        amp_gain=[34] * 3 + [151] * 3,
        spi_bus=1,
    )
    Logger.track_functions({
        "Fx": lambda: loadcell.fx,
        "Fy": lambda: loadcell.fy,
        "Fz": lambda: loadcell.fz,
        "Mx": lambda: loadcell.mx,
        "My": lambda: loadcell.my,
        "Mz": lambda: loadcell.mz,
    })

    with loadcell:
        loadcell.calibrate()
        for t in clock:
            loadcell.update()
            Logger.info(
                f"Time: {t}; Fx: {loadcell.fx}; Fy: {loadcell.fy}; Fz: {loadcell.fz};"
                f"Mx: {loadcell.mx}; My: {loadcell.my}; Mz: {loadcell.mz};"
            )
            Logger.record()


if __name__ == "__main__":
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="reading_loadcell_data.log",
    )

    clock = SoftRealtimeLoop(dt=DT)

    # demo_loadcell_actpack(loadcell_logger=loadcell_logger, clock=clock)
    # demo_loadcell_i2c(loadcell_logger=loadcell_logger, clock = clock)
    demo_loadcell_nb_daq(clock=clock)
