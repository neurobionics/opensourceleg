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


def home_joints():
    homing_logger = Logger(
        log_path="./logs",
        file_name="homing_joint",
    )

    clock = SoftRealtimeLoop(dt=DT)

    actuators = {
        "knee": DephyActuator(
            tag="knee",
            port="/dev/ttyACM0",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
        ),
        "ankle": DephyActuator(
            tag="ankle",
            port="/dev/ttyACM1",
            gear_ratio=GEAR_RATIO,
            frequency=FREQUENCY,
            dephy_log=False,
        ),
    }

    sensors = {
        "joint_encoder_knee": AS5048B(
            tag="joint_encoder_knee",
            bus="/dev/i2c-2",
            A1_adr_pin=True,
            A2_adr_pin=False,
            zero_position=0,
            enable_diagnostics=False,
        ),
        "joint_encoder_ankle": AS5048B(
            tag="joint_encoder_ankle",
            bus="/dev/i2c-3",
            A1_adr_pin=False,
            A2_adr_pin=True,
            zero_position=0,
            enable_diagnostics=False,
        ),
    }

    # Define callback functions for homing completion
    def knee_homing_complete():
        osl.joint_encoder_knee.update()
        osl.joint_encoder_knee.zero_position = osl.joint_encoder_knee.counts
        print("Knee homing complete!")

    def ankle_homing_complete():
        osl.joint_encoder_ankle.update()
        osl.joint_encoder_ankle.zero_position = osl.joint_encoder_ankle.counts + osl.joint_encoder_ankle.deg_to_counts(
            30
        )
        print("Ankle homing complete!")

    osl = OpenSourceLeg(
        tag="osl",
        actuators=actuators,
        sensors=sensors,
    )

    callbacks = {"knee": knee_homing_complete, "ankle": ankle_homing_complete}

    with osl:
        osl.home(
            homing_voltage=2000,
            homing_frequency=FREQUENCY,
            homing_direction={"knee": -1, "ankle": -1},
            output_position_offset={"knee": 0.0, "ankle": np.deg2rad(30.0)},
            current_threshold=5000,
            velocity_threshold=0.001,
            callbacks=callbacks,
        )

        osl.knee.set_control_mode(CONTROL_MODES.CURRENT)
        osl.knee.set_current_gains()

        osl.ankle.set_control_mode(CONTROL_MODES.CURRENT)
        osl.ankle.set_current_gains()

        osl.knee.set_output_torque(0)
        osl.ankle.set_output_torque(0)

        # homing_logger.set_stream_terminator("\r")
        for t in clock:
            osl.update()

            homing_logger.info(
                f"Time: {t:.2f}; Knee Output Position: {np.rad2deg(osl.knee.output_position):.2f}; "
                f"Ankle Output Position: {np.rad2deg(osl.ankle.output_position):.2f}; "
                f"Knee Encoder Position: {np.rad2deg(osl.joint_encoder_knee.position):.2f}; "
                f"Ankle Encoder Position: {np.rad2deg(osl.joint_encoder_ankle.position):.2f}; "
            )


if __name__ == "__main__":
    home_joints()
