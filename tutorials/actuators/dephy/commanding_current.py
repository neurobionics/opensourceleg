from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.rust import Logger
from opensourceleg.utilities import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 1000
DT = 1 / FREQUENCY
GEAR_RATIO = 1.0

CURRENT_SETPOINT = 600  # mA


def current_control():
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="current_control.log",
    )
    actpack = DephyActuator(
        port="/dev/ttyACM0", gear_ratio=GEAR_RATIO, frequency=FREQUENCY, debug_level=0, dephy_log=False
    )
    clock = SoftRealtimeLoop(dt=DT)

    # current_logger.set_stream_terminator("\r")

    with actpack:
        actpack.set_control_mode(mode=CONTROL_MODES.CURRENT)
        actpack.set_current_gains()

        command_current = 0

        Logger.track_functions({
            "Motor_Current": lambda: actpack.motor_current,
            "Command_Current": lambda: command_current,
        })

        for t in clock:
            if t > TIME_TO_STEP:
                command_current = CURRENT_SETPOINT
                actpack.set_motor_current(value=command_current)  # in mA

            actpack.update()

            Logger.info(
                f"Time: {t}; " f"Command Current: {command_current}; " f"Motor Current: {actpack.motor_current}",
            )
            Logger.record()


if __name__ == "__main__":
    current_control()
