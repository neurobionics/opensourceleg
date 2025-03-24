from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY
GEAR_RATIO = 9 * (83 / 18)


def home_joint():
    homing_logger = Logger(
        log_path="./logs",
        file_name="homing_joint",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0",
        tag="knee",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )
    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.home(
            homing_voltage=2000,
            homing_frequency=FREQUENCY,
            homing_direction=-1,
            joint_direction=-1,
            joint_position_offset=0.0,
            motor_position_offset=0.0,
            current_threshold=5000,
            velocity_threshold=0.001,
        )

        for t in clock:
            actpack.update()

            homing_logger.info(
                f"Time: {t}; " f"Joint Position: {actpack.joint_position}; " f"Motor Position: {actpack.motor_position}"
            )


if __name__ == "__main__":
    home_joint()
