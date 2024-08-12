import time

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
import pandas as pd

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0",
    gear_ratio=9.0,
)

with actpack:
    current_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Current": [],
            "Command_Current": [],
        }
    )
    iternum = 0
    time_period = 0.002
    try:
        actpack.set_control_mode(mode=actpack.CONTROL_MODES.CURRENT)
        actpack.set_current_gains(
            # if no input, then default gains are applied
        )

        for iternum in range(100):
            current = 0
            actpack.set_motor_current(value=current)  # in mA
            actpack.update()
            # iternum += 1
            current_data = pd.concat(
                [
                    current_data,
                    pd.DataFrame(
                        {
                            "Time": [iternum * time_period],
                            "Output_Current": [actpack.motor_current],
                            "Command_Current": [current],
                        }
                    ),
                ],
                ignore_index=True,
            )
            current_data.to_csv("current_data_dephy.csv", index=False)
            time.sleep(time_period)


        while True:
            iternum += 1
            current = 200
            actpack.set_motor_current(value=current)  # in mA
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
                            "Time": [iternum * time_period],
                            "Output_Current": [actpack.motor_current],
                            "Command_Current": [current],
                        }
                    ),
                ],
                ignore_index=True,
            )
            current_data.to_csv("current_data_dephy.csv", index=False)

            time.sleep(time_period)

    except KeyboardInterrupt:
        exit()
