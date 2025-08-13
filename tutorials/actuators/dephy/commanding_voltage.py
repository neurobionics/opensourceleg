import time

from opensourceleg_rs import Logger

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 1000
TIME_TO_STEP = 1.0
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0


def voltage_control():
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="voltage_control.log",
    )

    actpack = DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=GEAR_RATIO,
        frequency=FREQUENCY,
        debug_level=0,
        dephy_log=False,
    )

    command_voltage = 0

    Logger.track_functions({
        "Command_Voltage": lambda: command_voltage,
        "Motor_Voltage": lambda: actpack.motor_voltage,
        "Time": lambda: time.time(),
    })

    clock = SoftRealtimeLoop(dt=DT)

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

        for t in clock:
            actpack.update()

            if t > TIME_TO_STEP:
                command_voltage = 1000

            actpack.set_motor_voltage(value=command_voltage)

            Logger.info(
                f"Time: {t}; "
                f"Command Voltage: {command_voltage}; "
                f"Motor Voltage: {actpack.motor_voltage}; "
                f"Motor Current: {actpack.motor_current}",
            )
            Logger.record()


if __name__ == "__main__":
    voltage_control()
