import opensourceleg.actuators.dephy as Dephy
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 1000
TIME_TO_STEP = 1.0

if __name__ == "__main__":
    clock = SoftRealtimeLoop(dt=1 / FREQUENCY)
    actpack = Dephy.DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=1.0,
    )

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
        
        for t in clock:
            actpack.update()

            if t > TIME_TO_STEP:
                actpack.set_motor_voltage(value=1000)
            LOGGER.info(
                f"Motor Position: {actpack.motor_position}; "
                + f"Motor Voltage: {actpack.motor_voltage}; "
                + f"Motor Current: {actpack.motor_current}; "
            )
