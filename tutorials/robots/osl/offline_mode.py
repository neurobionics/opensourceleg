import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.osl import OpenSourceLeg
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
GEAR_RATIO = 9 * (83 / 18)


def OSL_offline_mode():
    logger = Logger(
        log_path="./logs",
        file_name="OSL_offline_mode",
    )

    clock = SoftRealtimeLoop(dt=DT)

    actuators = {
        "knee": DephyActuator(
            tag="knee",
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
            offline=True,
        ),
        "ankle": DephyActuator(
            tag="ankle",
            port="/dev/ttyACM1",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
            offline=True,
        ),
    }

    sensors = {
        "joint_encoder_knee": AS5048B(
            tag="joint_encoder_knee",
            bus=1,
            A1_adr_pin=True,
            A2_adr_pin=False,
            zero_position=0,
            enable_diagnostics=False,
            offline=True,
        ),
        "joint_encoder_ankle": AS5048B(
            tag="joint_encoder_ankle",
            bus=1,
            A1_adr_pin=False,
            A2_adr_pin=True,
            zero_position=0,
            enable_diagnostics=False,
            offline=True,
        ),
    }

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    with osl:
        osl.knee.set_control_mode(CONTROL_MODES.POSITION)
        osl.ankle.set_control_mode(CONTROL_MODES.POSITION)

        for t in clock:
            osl.update()

            osl.knee.set_output_position(2 * np.pi * 0.1 * t + np.pi / 4)
            osl.ankle.set_output_position(2 * np.pi * 0.1 * t)

            logger.info(
                f"Time: {t:.2f}; Knee Output Position: {np.rad2deg(osl.knee.output_position):.2f}; "
                f"Ankle Output Position: {np.rad2deg(osl.ankle.output_position):.2f}; "
                f"Knee Encoder Position: {np.rad2deg(osl.joint_encoder_knee.position):.2f}; "
                f"Ankle Encoder Position: {np.rad2deg(osl.joint_encoder_ankle.position):.2f}; "
            )


if __name__ == "__main__":
    OSL_offline_mode()
