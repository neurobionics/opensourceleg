import time

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.time import SoftRealtimeLoop

FREQUENCY = 1000
TIME_TO_STEP = 1.0
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0


def voltage_control():
    voltage_logger = Logger(
        log_path="./logs",
        file_name="voltage_control",
    )

    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )

    command_voltage = 0

    voltage_logger.track_variable(lambda: command_voltage, "Command Voltage")
    voltage_logger.track_variable(lambda: actpack.motor_voltage, "Motor Voltage")
    voltage_logger.track_variable(lambda: time.time(), "Time")

    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

        for t in clock:
            actpack.update()

            if t > TIME_TO_STEP:
                command_voltage = 1000

            actpack.set_motor_voltage(value=command_voltage)

            voltage_logger.info(
                f"Time: {t}; "
                f"Command Voltage: {command_voltage}; "
                f"Motor Voltage: {actpack.motor_voltage}; "
                f"Motor Current: {actpack.motor_current}",
            )
            voltage_logger.update()


if __name__ == "__main__":
    voltage_control()
