import time

import numpy as np
import pandas as pd

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY


def main():
    actpack = Dephy.DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=1.0,
    )

    position_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Position": [],
            "Command_Position": [],
        }
    )
    clock = SoftRealtimeLoop(dt=DT)
    with actpack:
        try:
            # actpack.start()
            actpack.set_control_mode(mode=actpack.CONTROL_MODES.POSITION)

            actpack.set_position_gains(
                # if no input, then default gains are applied
            )

            actpack.update()

            current_position = actpack.output_position
            for t in clock:

                if t > TIME_TO_STEP:
                    command_position = current_position + np.pi
                    actpack.set_output_position(value=command_position)
                else:
                    command_position = current_position

                actpack.update()

                LOGGER.info(
                    "".join(
                        f"Motor Position: {actpack.output_position}\t"
                        + f"Motor Voltage: {actpack.motor_voltage}\t"
                        + f"Motor Current: {actpack.motor_current}\t"
                    )
                )

                position_data = pd.concat(
                    [
                        position_data,
                        pd.DataFrame(
                            {
                                "Time": [t],
                                "Output_Position": [actpack.output_position],
                                "Command_Position": [command_position],
                            }
                        ),
                    ],
                    ignore_index=True,
                )

                # position_data.to_csv("position_data_dephy.csv", index=False)
                time.sleep(DT)

        finally:
            position_data.to_csv("position_data_dephy.csv", index=False)
            exit()


if __name__ == "__main__":
    main()
