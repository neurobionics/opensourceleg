import time

import pandas as pd

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY


def main():
    actpack = Dephy.DephyActpack(
        port="/dev/ttyACM0",
        gear_ratio=9.0,
    )
    current_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Current": [],
            "Command_Current": [],
        }
    )
    clock = SoftRealtimeLoop(dt=DT)
    with actpack:

        try:
            actpack.set_control_mode(mode=actpack.CONTROL_MODES.CURRENT)
            actpack.set_current_gains(
                # if no input, then default gains are applied
            )

            for t in clock:

                if t > TIME_TO_STEP:
                    command_current = 275
                    actpack.set_motor_current(value=command_current)  # in mA

                else:
                    command_current = 0

                actpack.update()

                LOGGER.info(
                    "".join(
                        f"Motor Position: {actpack.motor_position}\t"
                        + f"Motor Voltage: {actpack.motor_voltage}\t"
                        + f"Motor Current: {actpack.motor_current}\t"
                    )
                )
                current_data = pd.concat(
                    [
                        current_data,
                        pd.DataFrame(
                            {
                                "Time": [t],
                                "Output_Current": [actpack.motor_current],
                                "Command_Current": [command_current],
                            }
                        ),
                    ],
                    ignore_index=True,
                )

                time.sleep(DT)

        finally:
            current_data.to_csv("current_data_dephy.csv", index=False)
            exit()


if __name__ == "__main__":
    main()
