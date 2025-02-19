import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 1000

if __name__ == "__main__":
    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)
    actpack = Dephy.DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=1.0,
    )

    with actpack:
        for t in clock:
            actpack.update()
            LOGGER.info(
                f"Motor Position: {actpack.motor_position}; "
                + f"Motor Voltage: {actpack.motor_voltage}; "
                + f"Motor Current: {actpack.motor_current}; "
            )
